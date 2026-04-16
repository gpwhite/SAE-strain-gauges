#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
int release=1;
int version=1;
int subver =1;
constexpr uint8_t HX_COUNT = 3;
const uint8_t HX_DATA_PINS[HX_COUNT] = {4, 6, 14};
const uint8_t HX_CLOCK_PINS[HX_COUNT] = {5, 7, 15};
constexpr uint8_t LCD_ADDRESS = 0x27;
constexpr uint8_t LCD_COLUMNS = 20;
constexpr uint8_t LCD_ROWS = 4;
constexpr uint32_t SD_INIT_CLOCK = 1000000;
constexpr uint16_t ZERO_SAMPLE_COUNT = 32;
constexpr uint16_t SHUNT_SAMPLE_COUNT = 32;
constexpr float SHUNT_MICROSTRAIN = 10000.0f;
constexpr uint32_t LOG_INTERVAL_MS = 100;
const char *CALIBRATION_FILE = "/CALIB.TXT";
LiquidCrystal_PCF8574 lcd(LCD_ADDRESS);
File logFile;
char logFileName[16] = {0};
uint32_t lastLogAt = 0;
bool lcdReady = false;
struct Hx711Channel {
  uint8_t dataPin;
  uint8_t clockPin;
  void begin() const {
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, INPUT);
    digitalWrite(clockPin, LOW);
  }
  bool isReady() const {
    return digitalRead(dataPin) == LOW;
  }
  bool readRaw(long &value, uint32_t timeoutMs = 250) const {
    const uint32_t start = millis();
    while (!isReady()) {
      if (millis() - start > timeoutMs) {
        return false;
      }
      delay(1);
    }
    long reading = 0;
    noInterrupts();
    for (uint8_t i = 0; i < 24; ++i) {
      digitalWrite(clockPin, HIGH);
      delayMicroseconds(1);
      reading = (reading << 1) | digitalRead(dataPin);
      digitalWrite(clockPin, LOW);
      delayMicroseconds(1);
    }
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
    interrupts();
    if (reading & 0x800000L) {
      reading |= ~0xFFFFFFL;
    }
    value = reading;
    return true;
  }
};
struct CalibrationData {
  bool valid;
  float offset[HX_COUNT];
  float scale[HX_COUNT];
};
Hx711Channel channels[HX_COUNT] = {
  {HX_DATA_PINS[0], HX_CLOCK_PINS[0]},
  {HX_DATA_PINS[1], HX_CLOCK_PINS[1]},
  {HX_DATA_PINS[2], HX_CLOCK_PINS[2]}
};
CalibrationData calibration = {
  false,
  {0.0f, 0.0f, 0.0f},
  {1.0f, 1.0f, 1.0f}
};
bool probeLcd() {
  Wire.beginTransmission(LCD_ADDRESS);
  return Wire.endTransmission() == 0;
}
bool initLcd() {
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);
  delay(50);
  if (!probeLcd()) {
    return false;
  }
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.setBacklight(255);
  lcd.clear();
  lcdReady = true;
  return true;
}
void showMessage(const String &line1,
                 const String &line2 = "",
                 const String &line3 = "",
                 const String &line4 = "") {
  if (!lcdReady) {
    return;
  }
  lcd.clear();
  const String lines[4] = {line1, line2, line3, line4};
  for (uint8_t row = 0; row < LCD_ROWS; ++row) {
    lcd.setCursor(0, row);
    String text = lines[row];
    if (text.length() > LCD_COLUMNS) {
      text = text.substring(0, LCD_COLUMNS);
    }
    lcd.print(text);
    for (uint8_t col = text.length(); col < LCD_COLUMNS; ++col) {
      lcd.print(' ');
    }
  }
}
void haltForever() {
  while (true) {
    delay(1000);
  }
}
bool captureAverage(uint16_t sampleCount, long averages[HX_COUNT]) {
  int64_t sums[HX_COUNT] = {0, 0, 0};
  for (uint16_t sample = 0; sample < sampleCount; ++sample) {
    for (uint8_t channel = 0; channel < HX_COUNT; ++channel) {
      long raw = 0;
      if (!channels[channel].readRaw(raw)) {
        return false;
      }
      sums[channel] += raw;
    }
  }
  for (uint8_t channel = 0; channel < HX_COUNT; ++channel) {
    averages[channel] = static_cast<long>(sums[channel] / sampleCount);
  }
  return true;
}
bool saveCalibration(const CalibrationData &data) {
  if (SD.exists(CALIBRATION_FILE)) {
    SD.remove(CALIBRATION_FILE);
  }
  File file = SD.open(CALIBRATION_FILE, FILE_WRITE);
  if (!file) {
    return false;
  }
  file.println("valid=1");
  for (uint8_t channel = 0; channel < HX_COUNT; ++channel) {
    file.print("offset");
    file.print(channel + 1);
    file.print('=');
    file.println(data.offset[channel], 3);
    file.print("scale");
    file.print(channel + 1);
    file.print('=');
    file.println(data.scale[channel], 9);
  }
  file.close();
  return true;
}
bool loadCalibration(CalibrationData &data) {
  if (!SD.exists(CALIBRATION_FILE)) {
    return false;
  }
  File file = SD.open(CALIBRATION_FILE, FILE_READ);
  if (!file) {
    return false;
  }
  CalibrationData loaded = {
    false,
    {0.0f, 0.0f, 0.0f},
    {1.0f, 1.0f, 1.0f}
  };
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      continue;
    }
    int separator = line.indexOf('=');
    if (separator < 0) {
      continue;
    }
    String key = line.substring(0, separator);
    String value = line.substring(separator + 1);
    if (key == "valid" && value == "1") {
      loaded.valid = true;
      continue;
    }
    for (uint8_t channel = 0; channel < HX_COUNT; ++channel) {
      if (key == ("offset" + String(channel + 1))) {
        loaded.offset[channel] = value.toFloat();
      }
      if (key == ("scale" + String(channel + 1))) {
        loaded.scale[channel] = value.toFloat();
      }
    }
  }
  file.close();
  if (!loaded.valid) {
    return false;
  }
  data = loaded;
  return true;
}
bool openNextLogFile() {
  for (uint16_t index = 0; index < 1000; ++index) {
    snprintf(logFileName, sizeof(logFileName), "/LOG%03u.CSV", index);
    if (!SD.exists(logFileName)) {
      logFile = SD.open(logFileName, FILE_WRITE);
      if (!logFile) {
        return false;
      }
      logFile.println("millis,strain_1_ue,strain_2_ue,strain_3_ue,raw_1,raw_2,raw_3");
      logFile.flush();
      return true;
    }
  }
  return false;
}
bool initSdCard() {
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin(SCK, MISO, MOSI, SS);
  delay(10);
  if (!SD.begin(SS, SPI, SD_INIT_CLOCK, "/sd", 5, false)) {
    return false;
  }
  return true;
}
bool runCalibration() {
  long zeroAverages[HX_COUNT] = {0, 0, 0};
  long shuntAverages[HX_COUNT] = {0, 0, 0};
  showMessage("Calibration", "Leave gauges still", "Capturing zero...");
  delay(3000);
  if (!captureAverage(ZERO_SAMPLE_COUNT, zeroAverages)) {
    showMessage("Zero capture fail", "Check HX711 wiring");
    return false;
  }
  showMessage("Calibration", "Press shunt buttons", "Then wait...");
  delay(5000);
  if (!captureAverage(SHUNT_SAMPLE_COUNT, shuntAverages)) {
    showMessage("Shunt capture fail", "Check modules");
    return false;
  }
  for (uint8_t channel = 0; channel < HX_COUNT; ++channel) {
    float delta = static_cast<float>(shuntAverages[channel] - zeroAverages[channel]);
    if (fabs(delta) < 1.0f) {
      showMessage("Bad calibration", "Delta too small", "Channel " + String(channel + 1));
      return false;
    }
    calibration.offset[channel] = static_cast<float>(zeroAverages[channel]);
    calibration.scale[channel] = SHUNT_MICROSTRAIN / delta;
  }
  calibration.valid = true;
  saveCalibration(calibration);
  showMessage("Calibration done", "Logging to SD", logFileName);
  delay(1500);
  return true;
}
float convertToMicrostrain(uint8_t channel, long raw) {
  return (static_cast<float>(raw) - calibration.offset[channel]) * calibration.scale[channel];
}
bool captureCurrentReadings(long rawReadings[HX_COUNT], float strains[HX_COUNT]) {
  for (uint8_t channel = 0; channel < HX_COUNT; ++channel) {
    if (!channels[channel].readRaw(rawReadings[channel])) {
      return false;
    }
    strains[channel] = convertToMicrostrain(channel, rawReadings[channel]);
  }
  return true;
}
void showLiveReadings(const float strains[HX_COUNT]) {
  if (!lcdReady) {
    return;
  }
  lcd.setCursor(0, 0);
  lcd.print("S1:");
  lcd.print(strains[0], 0);
  lcd.print(" ue     ");
  lcd.setCursor(0, 1);
  lcd.print("S2:");
  lcd.print(strains[1], 0);
  lcd.print(" ue     ");
  lcd.setCursor(0, 2);
  lcd.print("S3:");
  lcd.print(strains[2], 0);
  lcd.print(" ue     ");
  lcd.setCursor(0, 3);
  lcd.print("Log:");
  lcd.print(logFileName);
  lcd.print("   ");
}
void logReadings(uint32_t now, const long rawReadings[HX_COUNT], const float strains[HX_COUNT]) {
  if (!logFile) {
    return;
  }
  logFile.print(now);
  logFile.print(',');
  logFile.print(strains[0], 2);
  logFile.print(',');
  logFile.print(strains[1], 2);
  logFile.print(',');
  logFile.print(strains[2], 2);
  logFile.print(',');
  logFile.print(rawReadings[0]);
  logFile.print(',');
  logFile.print(rawReadings[1]);
  logFile.print(',');
  logFile.println(rawReadings[2]);
  logFile.flush();
}
void setup() {
  for (uint8_t channel = 0; channel < HX_COUNT; ++channel) {
    channels[channel].begin();
  }
  if (!initLcd()) {
    haltForever();
  }
//Show init message
  showMessage("UB SAE DAQ Box", "Version: " + String(release)+"."+String(version)+"."+String(subver),
              "Created By: GPWhite", "Initializing...");
  delay(11500);

//Handle sd card
  if (!initSdCard()) {
    showMessage("SD init failed", "CS=" + String(SS) + " CLK=" + String(SCK),
                "MOSI=" + String(MOSI) + " MISO=" + String(MISO), "Check SPI wiring");
    haltForever();
  }

//Heres where we get to the shunt calibration
  loadCalibration(calibration);
  if (!openNextLogFile()) {
    showMessage("Log open failed", "Check SD card");
    haltForever();
  }
  if (!runCalibration()) {
    showMessage("Calibration failed", "Power cycle to retry");
    haltForever();
  }
}
void loop() {
  if (!calibration.valid) {
    delay(1000);
    return;
  }
  uint32_t now = millis();
  if (now - lastLogAt < LOG_INTERVAL_MS) {
    delay(5);
    return;
  }
  lastLogAt = now;
  long rawReadings[HX_COUNT] = {0, 0, 0};
  float strains[HX_COUNT] = {0.0f, 0.0f, 0.0f};
  if (!captureCurrentReadings(rawReadings, strains)) {
    showMessage("Read error", "Check HX711 input");
    delay(500);
    return;
  }
  showLiveReadings(strains);
  logReadings(now, rawReadings, strains);
}
