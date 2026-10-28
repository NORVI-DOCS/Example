/*
 * NORVI X-CPU-ESPS3-X3  --  Board self-test (+ optional X-AI4 4-20 mA read)
 *
 * Open the Serial Monitor at 115200 baud. Tests run through single-key
 * commands. Nothing drives an output unless you ask for it ('o').
 *
 *   a  run all safe tests (info, I2C, RTC, LEDs, SD, WiFi)
 *   i  chip / flash / PSRAM info
 *   s  I2C scan + expected-device check
 *   t  RTC (DS3231) test          T  set RTC to this sketch's build time
 *   l  blink the two onboard LEDs
 *   b  button monitor for 10 s (press PB1 / PB3)
 *   d  SD card test (write / read / delete a file)
 *   w  WiFi scan test
 *   g  GSM/4G modem AT test (resets the modem, takes up to ~15 s)
 *   r  RS485 echo test
 *   o  OUTPUT TEST - switches every output ON then OFF in turn
 *      (DISCONNECT LOADS FIRST)
 *   c  read two 4-20 mA sensors on an X-AI4 (inputs 1 and 2, address 0x49)
 *   ?  show this menu
 *
 * Libraries: Wire, SPI, FS, SD, WiFi (ESP32 core) + Adafruit ADS1X15 / BusIO.
 * Not covered here (need extra libraries): TFT display, touch, Ethernet.
 */

#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <WiFi.h>
#include <time.h>
#include <Adafruit_ADS1X15.h>

// ---------------- Pins (NORVI X3) ----------------
#define SDA_PIN    8
#define SCL_PIN    9
#define SPI_SCLK   12
#define SPI_MISO   13
#define SPI_MOSI   11
#define SD_CS      42
#define ETH_CS     1
#define TFT_CS     45
#define PORT_RST   38      // HIGH = release the PCA9539 IO expander
#define RS485_RXD  16
#define RS485_TXD  15
#define GSM_RX     18
#define GSM_TX     17
#define GSM_RST    39

// ---------------- I2C devices ----------------
#define ADDR_TOUCH   0x15  // CST816S
#define ADDR_PCA9536 0x41  // buttons + LEDs
#define ADDR_RTC     0x68  // DS3231
#define ADDR_PCA9539 0x75  // 16-bit output expander
#define ADDR_AI4     0x49  // X-AI4 (optional)

// PCA9536 pins
#define IO_PB1   0
#define IO_LED1  1
#define IO_LED2  2
#define IO_PB3   3         // LEDs are active LOW

// Direct-GPIO outputs (OUTPUT11/12/13 = GPIO 35/36/37 are skipped: not usable with PSRAM)
const uint8_t  gpioOutputs[]    = {5, 6, 7, 10, 14, 21, 40};
// PCA9539 port-1 outputs: OUTPUT19, 20, 33, 34, 38  -> port1 bits 1..5
const char*    pcaOutputNames[] = {"OUTPUT19", "OUTPUT20", "OUTPUT33", "OUTPUT34", "OUTPUT38"};

Adafruit_ADS1115 ads;
const float mA_Factor = 4.096 / 3269.826;   // from the NORVI X-AI4 example

int g_pass = 0, g_fail = 0, g_warn = 0;
uint8_t pca9536Out = 0x06;                  // LEDs off (active low)

// ---------------- helpers ----------------
void report(const char* name, int status, const String& detail = "") {
  // status: 1 = PASS, 0 = FAIL, 2 = WARN/INFO
  if (status == 1)      { g_pass++; Serial.print("  [PASS] "); }
  else if (status == 0) { g_fail++; Serial.print("  [FAIL] "); }
  else                  { g_warn++; Serial.print("  [WARN] "); }
  Serial.print(name);
  if (detail.length()) { Serial.print(" - "); Serial.print(detail); }
  Serial.println();
}

bool i2cPresent(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

bool i2cWriteReg(uint8_t a, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

bool i2cReadReg(uint8_t a, uint8_t reg, uint8_t &v) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 1) != 1) return false;
  v = Wire.read();
  return true;
}

uint8_t bcd2dec(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }
uint8_t dec2bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10); }

void beginTests(const char* title) {
  Serial.println();
  Serial.print("== "); Serial.print(title); Serial.println(" ==");
}

void printSummary() {
  Serial.println();
  Serial.printf("RESULT: %d passed, %d failed, %d warnings\n", g_pass, g_fail, g_warn);
}

// ---------------- LEDs / buttons (PCA9536) ----------------
void setLed(uint8_t led, bool on) {
  uint8_t bit = (led == 1) ? IO_LED1 : IO_LED2;
  if (on) pca9536Out &= ~(1 << bit); else pca9536Out |= (1 << bit);
  i2cWriteReg(ADDR_PCA9536, 0x01, pca9536Out);
}

bool readButtons(uint8_t &pb1, uint8_t &pb3) {
  uint8_t v;
  if (!i2cReadReg(ADDR_PCA9536, 0x00, v)) return false;
  pb1 = (v >> IO_PB1) & 1;
  pb3 = (v >> IO_PB3) & 1;
  return true;
}

void initExpanders() {
  // PCA9536: P0 and P3 inputs (buttons), P1 and P2 outputs (LEDs)
  i2cWriteReg(ADDR_PCA9536, 0x01, pca9536Out);
  i2cWriteReg(ADDR_PCA9536, 0x03, 0x09);
  // PCA9539: only port-1 bits 1..5 become outputs (OUTPUT19..38), all LOW.
  // CS1..CS4 are left as inputs so the expansion chip-selects stay deselected.
  i2cWriteReg(ADDR_PCA9539, 0x03, 0x00);
  i2cWriteReg(ADDR_PCA9539, 0x07, 0xC1);
}

// ---------------- tests ----------------
void testInfo() {
  beginTests("Chip info");
  Serial.printf("  Chip: %s rev %d, %d core(s) @ %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz());
  uint32_t flash = ESP.getFlashChipSize();
  report("Flash size", flash >= 16UL * 1024 * 1024 ? 1 : 0, String(flash / (1024 * 1024)) + " MB (expected 16 MB)");
  if (psramFound()) report("PSRAM", 1, String(ESP.getPsramSize() / 1024) + " KB");
  else report("PSRAM", 2, "not enabled - set PSRAM = QSPI PSRAM in the board options");
  Serial.printf("  Free heap: %u bytes\n", ESP.getFreeHeap());
}

void testI2C() {
  beginTests("I2C scan (SDA 8 / SCL 9)");
  Serial.print("  Found:");
  int count = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) { Serial.printf(" 0x%02X", a); count++; }
  }
  if (count == 0) Serial.print(" nothing");
  Serial.println();
  report("Touch controller CST816S (0x15)", i2cPresent(ADDR_TOUCH) ? 1 : 0);
  report("PCA9536 buttons/LEDs (0x41)",     i2cPresent(ADDR_PCA9536) ? 1 : 0);
  report("DS3231 RTC (0x68)",               i2cPresent(ADDR_RTC) ? 1 : 0);
  report("PCA9539 IO expander (0x75)",      i2cPresent(ADDR_PCA9539) ? 1 : 0);
  if (i2cPresent(ADDR_AI4)) report("X-AI4 module (0x49)", 1, "detected");
  else Serial.println("  (X-AI4 at 0x49 not detected - only needed for the 'c' command)");
}

void readRtcTime(char *buf, size_t n, bool &valid) {
  uint8_t r[7];
  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADDR_RTC, 7);
  for (int i = 0; i < 7; i++) r[i] = Wire.available() ? Wire.read() : 0;
  uint8_t sec = bcd2dec(r[0] & 0x7F), mn = bcd2dec(r[1]), hr = bcd2dec(r[2] & 0x3F);
  uint8_t d = bcd2dec(r[4]), mo = bcd2dec(r[5] & 0x1F), y = bcd2dec(r[6]);
  snprintf(buf, n, "20%02d-%02d-%02d %02d:%02d:%02d", y, mo, d, hr, mn, sec);
  valid = (mo >= 1 && mo <= 12 && d >= 1 && d <= 31 && hr < 24 && mn < 60 && sec < 60);
}

void testRTC() {
  beginTests("RTC DS3231");
  if (!i2cPresent(ADDR_RTC)) { report("RTC present", 0, "no ACK at 0x68"); return; }
  char t1[24], t2[24];
  bool v1, v2;
  readRtcTime(t1, sizeof(t1), v1);
  delay(1200);
  readRtcTime(t2, sizeof(t2), v2);
  Serial.printf("  Time now: %s\n", t2);
  report("RTC counting", strcmp(t1, t2) != 0 ? 1 : 0, strcmp(t1, t2) != 0 ? "seconds advanced" : "time did not change");
  uint8_t status = 0;
  i2cReadReg(ADDR_RTC, 0x0F, status);
  if (status & 0x80) report("RTC time valid", 2, "oscillator stopped earlier (battery?) - send 'T' to set the time");
  else report("RTC time valid", v2 ? 1 : 2, v2 ? "" : "date looks invalid - send 'T' to set the time");
  uint8_t th = 0, tl = 0;
  if (i2cReadReg(ADDR_RTC, 0x11, th) && i2cReadReg(ADDR_RTC, 0x12, tl)) {
    float temp = (int8_t)th + ((tl >> 6) * 0.25f);
    report("RTC temperature", (temp > -20 && temp < 85) ? 1 : 0, String(temp, 2) + " C");
  }
}

void setRtcToBuildTime() {
  beginTests("Set RTC to build time");
  char mon[4]; int d, y, hh, mm, ss;
  sscanf(__DATE__, "%3s %d %d", mon, &d, &y);
  sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  int mo = (strstr(months, mon) - months) / 3 + 1;
  struct tm tmv = {};
  tmv.tm_year = y - 1900; tmv.tm_mon = mo - 1; tmv.tm_mday = d;
  tmv.tm_hour = hh; tmv.tm_min = mm; tmv.tm_sec = ss;
  mktime(&tmv);
  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x00);
  Wire.write(dec2bcd(ss)); Wire.write(dec2bcd(mm)); Wire.write(dec2bcd(hh));
  Wire.write(tmv.tm_wday + 1); Wire.write(dec2bcd(d)); Wire.write(dec2bcd(mo)); Wire.write(dec2bcd(y % 100));
  bool ok = Wire.endTransmission() == 0;
  uint8_t status = 0;
  if (i2cReadReg(ADDR_RTC, 0x0F, status)) i2cWriteReg(ADDR_RTC, 0x0F, status & 0x7F);
  report("RTC set", ok ? 1 : 0, String(__DATE__) + " " + __TIME__ + " (build time, not live time)");
}

void testLEDs() {
  beginTests("Onboard LEDs");
  if (!i2cPresent(ADDR_PCA9536)) { report("PCA9536", 0, "not found"); return; }
  Serial.println("  Watch LED1 and LED2 blink alternately...");
  for (int i = 0; i < 3; i++) {
    setLed(1, true);  setLed(2, false); delay(300);
    setLed(1, false); setLed(2, true);  delay(300);
  }
  setLed(1, false); setLed(2, false);
  report("LED control", 2, "check visually that both LEDs blinked");
}

void testButtons() {
  beginTests("Buttons (10 s) - press PB1 and PB3");
  uint8_t p1, p3, l1 = 255, l3 = 255;
  bool seen1 = false, seen3 = false;
  uint8_t base1 = 255, base3 = 255;
  unsigned long t0 = millis();
  while (millis() - t0 < 10000) {
    if (!readButtons(p1, p3)) { report("PCA9536 read", 0); return; }
    if (base1 == 255) { base1 = p1; base3 = p3; }
    if (p1 != l1 || p3 != l3) {
      Serial.printf("  PB1=%d  PB3=%d\n", p1, p3);
      l1 = p1; l3 = p3;
    }
    if (p1 != base1) seen1 = true;
    if (p3 != base3) seen3 = true;
    delay(20);
  }
  report("PB1 changed state", seen1 ? 1 : 2, seen1 ? "" : "not pressed during the test");
  report("PB3 changed state", seen3 ? 1 : 2, seen3 ? "" : "not pressed during the test");
}

void testSD() {
  beginTests("SD card");
  pinMode(TFT_CS, OUTPUT); digitalWrite(TFT_CS, HIGH);   // deselect other SPI devices
  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH);
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
  if (!SD.begin(SD_CS, SPI, 10000000)) { report("SD mount", 0, "no card / card not readable"); return; }
  uint8_t type = SD.cardType();
  if (type == CARD_NONE) { report("SD card", 0, "no card detected"); SD.end(); return; }
  report("SD mount", 1, String((uint32_t)(SD.cardSize() / (1024 * 1024))) + " MB");
  File f = SD.open("/x3_test.txt", FILE_WRITE);
  if (!f) { report("SD write", 0); SD.end(); return; }
  f.print("NORVI X3 SD TEST");
  f.close();
  f = SD.open("/x3_test.txt");
  String back = f ? f.readString() : "";
  if (f) f.close();
  report("SD write/read", back == "NORVI X3 SD TEST" ? 1 : 0);
  SD.remove("/x3_test.txt");
  SD.end();
}

void testWiFi() {
  beginTests("WiFi scan");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  if (n < 0)       report("WiFi radio", 0, "scan failed");
  else if (n == 0) report("WiFi radio", 2, "scan ok but no networks found");
  else {
    report("WiFi radio", 1, String(n) + " network(s) found");
    for (int i = 0; i < n && i < 3; i++)
      Serial.printf("    %s (%d dBm)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
  }
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
}

void testModem() {
  beginTests("GSM / 4G modem (AT)");
  Serial1.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  pinMode(GSM_RST, OUTPUT);
  digitalWrite(GSM_RST, LOW);  delay(200);
  digitalWrite(GSM_RST, HIGH);
  Serial.println("  Modem reset, waiting for it to answer (up to 15 s)...");
  delay(2000);
  String resp;
  bool ok = false;
  for (int i = 0; i < 12 && !ok; i++) {
    while (Serial1.available()) Serial1.read();
    Serial1.print("AT\r\n");
    unsigned long t0 = millis();
    resp = "";
    while (millis() - t0 < 1000) {
      while (Serial1.available()) resp += (char)Serial1.read();
      if (resp.indexOf("OK") >= 0) { ok = true; break; }
    }
  }
  report("Modem AT", ok ? 1 : 0, ok ? "answered OK" : "no answer (modem fitted and powered?)");
  if (ok) {
    while (Serial1.available()) Serial1.read();
    Serial1.print("ATI\r\n");
    delay(500);
    String info;
    while (Serial1.available()) info += (char)Serial1.read();
    info.trim();
    Serial.println("  ATI: " + info);
  }
}

void testRS485() {
  beginTests("RS485");
  Serial2.begin(115200, SERIAL_8N1, RS485_RXD, RS485_TXD);
  delay(50);
  while (Serial2.available()) Serial2.read();
  const char* msg = "RS485 TEST";
  Serial2.println(msg);
  delay(300);
  String rx;
  while (Serial2.available()) rx += (char)Serial2.read();
  if (rx.indexOf(msg) >= 0) report("RS485 echo", 1, "received what was sent");
  else report("RS485 echo", 2, "nothing received - connect A/B to another RS485 device or a loopback to verify");
  Serial2.end();
}

void testOutputs() {
  beginTests("OUTPUT TEST (loads must be disconnected)");
  if (!i2cPresent(ADDR_PCA9539)) report("PCA9539", 0, "not found - OUTPUT19/20/33/34/38 skipped");
  for (size_t i = 0; i < sizeof(gpioOutputs); i++) {
    uint8_t p = gpioOutputs[i];
    pinMode(p, OUTPUT);
    Serial.printf("  OUTPUT%d (GPIO %d) ON\n", p, p);
    digitalWrite(p, HIGH); delay(400);
    digitalWrite(p, LOW);  delay(150);
  }
  if (i2cPresent(ADDR_PCA9539)) {
    for (int i = 0; i < 5; i++) {
      uint8_t bit = i + 1;
      Serial.printf("  %s ON\n", pcaOutputNames[i]);
      i2cWriteReg(ADDR_PCA9539, 0x03, 1 << bit);
      delay(100);
      uint8_t rb = 0;
      bool ok = i2cReadReg(ADDR_PCA9539, 0x03, rb) && (rb & (1 << bit));
      if (!ok) report(pcaOutputNames[i], 0, "output register did not take the value");
      delay(300);
      i2cWriteReg(ADDR_PCA9539, 0x03, 0x00);
      delay(150);
    }
  }
  report("Output sequence", 2, "check each output with a meter / indicator");
}

void testAI4() {
  beginTests("X-AI4: two 4-20 mA sensors (inputs 1 and 2)");
  if (!ads.begin(ADDR_AI4)) { report("ADS1115", 0, "not found at 0x49 (DIP1 ON, others OFF)"); return; }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);
  for (int n = 0; n < 5; n++) {
    float mA1 = ads.readADC_SingleEnded(0) * mA_Factor;
    float mA2 = ads.readADC_SingleEnded(1) * mA_Factor;
    Serial.printf("  Sensor 1: %6.2f mA%s   Sensor 2: %6.2f mA%s\n",
                  mA1, mA1 < 3.5 ? " (no signal)" : "",
                  mA2, mA2 < 3.5 ? " (no signal)" : "");
    delay(500);
  }
  report("X-AI4 read", 1, "values above; below 3.5 mA means no loop current");
}

void runAll() {
  g_pass = g_fail = g_warn = 0;
  testInfo();
  testI2C();
  testRTC();
  testLEDs();
  testSD();
  testWiFi();
  printSummary();
}

void printMenu() {
  Serial.println();
  Serial.println("NORVI X3 self-test  (send a key)");
  Serial.println("  a all safe tests   i chip info   s I2C scan   t RTC   T set RTC to build time");
  Serial.println("  l LEDs   b buttons   d SD card   w WiFi   g modem AT   r RS485");
  Serial.println("  o OUTPUT test (disconnect loads!)   c X-AI4 4-20 mA   ? menu");
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(300);
  Serial.println();
  Serial.println("NORVI X-CPU-ESPS3-X3 self-test");

  pinMode(PORT_RST, OUTPUT);
  digitalWrite(PORT_RST, HIGH);           // enable the PCA9539
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  initExpanders();

  runAll();
  printMenu();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'a': runAll(); break;
      case 'i': g_pass = g_fail = g_warn = 0; testInfo(); printSummary(); break;
      case 's': g_pass = g_fail = g_warn = 0; testI2C();  printSummary(); break;
      case 't': g_pass = g_fail = g_warn = 0; testRTC();  printSummary(); break;
      case 'T': setRtcToBuildTime(); break;
      case 'l': g_pass = g_fail = g_warn = 0; testLEDs(); printSummary(); break;
      case 'b': g_pass = g_fail = g_warn = 0; testButtons(); printSummary(); break;
      case 'd': g_pass = g_fail = g_warn = 0; testSD();   printSummary(); break;
      case 'w': g_pass = g_fail = g_warn = 0; testWiFi(); printSummary(); break;
      case 'g': g_pass = g_fail = g_warn = 0; testModem(); printSummary(); break;
      case 'r': g_pass = g_fail = g_warn = 0; testRS485(); printSummary(); break;
      case 'o': g_pass = g_fail = g_warn = 0; testOutputs(); printSummary(); break;
      case 'c': g_pass = g_fail = g_warn = 0; testAI4();  printSummary(); break;
      case '?': printMenu(); break;
      default: break;
    }
  }
  delay(10);
}
