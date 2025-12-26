#include <Wire.h>
#include <Adafruit_PN532.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <RTClib.h>
#include <esp_task_wdt.h> // Hardware Watchdog

// =================== 1. HARDWARE PINS ===================
// TFT Display (VSPI)
#define TFT_SCK  18
#define TFT_MOSI 23
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST  15

// NFC Reader (I2C)
#define PN532_SDA 21
#define PN532_SCL 22

// Servo & FPGA Control
#define SERVO_PIN 17
#define BUS_CLK    4   // FPGA Clock
#define BUS_LATCH 16   // FPGA Latch

// FPGA Data Bus (Parallel)
#define BUS_D0 25
#define BUS_D1 26
#define BUS_D2 27
#define BUS_D3 32
#define BUS_D4 33
#define BUS_D5 12
#define BUS_D6 13
#define BUS_D7 14

// FPGA Status Inputs
#define FPGA_VALID  34
#define FPGA_REPLAY 35

// =================== 2. SETTINGS & CONSTANTS ===================
#define EEPROM_ADDR 0x57  // I2C Address of AT24C32
#define MAX_LOGS    100   // Keep last 100 entries
#define LOG_SIZE    10    // Bytes per log entry
#define WDT_TIMEOUT 5     // Reboot if system freezes for 5 seconds

// Color Swap Fix (For your specific BGR screen)
#define COLOR_BLUE    ST7735_RED
#define COLOR_RED     ST7735_BLUE
#define COLOR_GREEN   ST7735_GREEN
#define COLOR_YELLOW  ST7735_YELLOW
#define COLOR_BLACK   ST7735_BLACK
#define COLOR_WHITE   ST7735_WHITE

// =================== 3. OBJECTS & GLOBALS ===================
RTC_DS3231 rtc;
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_PN532 pn532(PN532_SDA, PN532_SCL);
Servo doorServo;

// Authorized Tags
String allowedTags[] = { "B5F84306", "9C27F905" };
int numTags = 2;

// System State
uint32_t authCounter = 0;
int logIndex = 0;
unsigned long lastClockUpdate = 0;
bool needsScreenReset = true;

// =================== 4. MEMORY & LOGGING FUNCTIONS ===================

// Write single byte to I2C EEPROM
void writeEEPROM(unsigned int address, byte data) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5); // Write cycle delay
}

// Log an event (Time + UID)
void logAccess(String uid) {
  DateTime now = rtc.now();
  int startAddr = logIndex * LOG_SIZE;

  // Write Time (3 bytes)
  writeEEPROM(startAddr + 0, now.hour());
  writeEEPROM(startAddr + 1, now.minute());
  writeEEPROM(startAddr + 2, now.second());

  // Write UID (4 bytes)
  for(int i=0; i<4; i++) {
    char c = (i < uid.length()) ? uid[i] : 0;
    writeEEPROM(startAddr + 3 + i, c);
  }

  Serial.print("LOGGED: "); Serial.println(now.timestamp());
  
  logIndex++;
  if (logIndex >= MAX_LOGS) logIndex = 0; // Ring buffer
}

// Read and Print all logs to Serial Monitor
void printLogs() {
  Serial.println("\n--- ACCESS LOGS (FROM EEPROM) ---");
  for (int i = 0; i < MAX_LOGS; i++) {
    int startAddr = i * LOG_SIZE;
    
    // Check if entry exists (read Hour byte)
    byte h = 0; 
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((int)(startAddr >> 8));
    Wire.write((int)(startAddr & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(EEPROM_ADDR, 1);
    if (Wire.available()) h = Wire.read();
    
    // 255 (0xFF) means empty/erased memory
    if (h == 255) break; 

    // Read remaining bytes (Minute, Second)
    byte m = 0, s = 0;
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((int)((startAddr+1) >> 8));
    Wire.write((int)((startAddr+1) & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(EEPROM_ADDR, 2);
    if (Wire.available() >= 2) { m = Wire.read(); s = Wire.read(); }

    // Print Timestamp
    Serial.print("Log #"); Serial.print(i);
    Serial.print(": ");
    if (h<10) Serial.print("0"); Serial.print(h); Serial.print(":");
    if (m<10) Serial.print("0"); Serial.print(m); Serial.print(":");
    if (s<10) Serial.print("0"); Serial.print(s);
    Serial.print(" - UID: ");
    
    // Read and Print UID Chars
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((int)((startAddr+3) >> 8));
    Wire.write((int)((startAddr+3) & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(EEPROM_ADDR, 4);
    for(int j=0; j<4; j++) {
      if(Wire.available()) Serial.print((char)Wire.read());
    }
    Serial.println();
  }
  Serial.println("--- END OF LOGS ---\n");
}

// =================== 5. UI & HELPER FUNCTIONS ===================

// Draw the HH:MM:SS clock on the idle screen
void drawClock() {
  DateTime now = rtc.now();
  
  // Position specifically to overwrite previous time
  tft.setCursor(15, 60);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_WHITE, COLOR_BLUE); // White text on Blue BG
  
  char timeBuffer[10];
  sprintf(timeBuffer, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  tft.print(timeBuffer);
}

// Reset the screen to "Ready" state
void resetIdleScreen() {
  tft.fillScreen(COLOR_BLUE);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.setTextColor(COLOR_BLACK);
  tft.print("SECURE SYSTEM");
  
  tft.setCursor(5, 40);
  tft.setTextSize(2);
  tft.print("READY...");
  
  needsScreenReset = false;
  drawClock(); // Draw time immediately
}

// Show a status message (Granted/Denied)
void updateStatus(const char *l1, const char *l2, uint16_t color) {
  tft.fillScreen(color);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.setTextColor(COLOR_BLACK);
  tft.print(l1);
  
  tft.setTextSize(2);
  tft.setCursor(5, 40);
  tft.print(l2);
  
  needsScreenReset = true; // Mark that idle screen needs rebuilding
}

bool isAllowed(const String &tag) {
  for (int i = 0; i < numTags; i++) {
    if (tag == allowedTags[i]) return true;
  }
  return false;
}

// =================== 6. FPGA COMMUNICATION FUNCTIONS ===================

void setBusByte(uint8_t b) {
  digitalWrite(BUS_D0, b & 0x01);
  digitalWrite(BUS_D1, (b >> 1) & 0x01);
  digitalWrite(BUS_D2, (b >> 2) & 0x01);
  digitalWrite(BUS_D3, (b >> 3) & 0x01);
  digitalWrite(BUS_D4, (b >> 4) & 0x01);
  digitalWrite(BUS_D5, (b >> 5) & 0x01);
  digitalWrite(BUS_D6, (b >> 6) & 0x01);
  digitalWrite(BUS_D7, (b >> 7) & 0x01);
}

void sendTokenToFPGA(uint64_t token) {
  // Send 8 bytes, MSB first
  for (int i = 7; i >= 0; i--) {
    uint8_t b = (token >> (i * 8)) & 0xFF;
    setBusByte(b);
    
    // Clock Pulse
    digitalWrite(BUS_CLK, HIGH); delayMicroseconds(5);
    digitalWrite(BUS_CLK, LOW);  delayMicroseconds(5);
  }
  // Latch Pulse
  digitalWrite(BUS_LATCH, HIGH); delayMicroseconds(10);
  digitalWrite(BUS_LATCH, LOW);
}

uint64_t generateToken(const String &tag) {
  uint32_t tagHash = 0;
  for (char c : tag) tagHash = (tagHash << 5) + (uint8_t)c;
  return ((uint64_t)tagHash << 32) | authCounter;
}

// =================== 7. MAIN SETUP ===================

void setup() {
  Serial.begin(115200);

  // --- WATCHDOG SETUP (ESP32 Board v3.0+) ---
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = (1 << 0) | (1 << 1),
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); 
  // ------------------------------------------

  // --- TFT INIT ---
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS); 
  tft.initR(INITR_GREENTAB); 
  tft.setRotation(1);

  // --- I2C / NFC / RTC INIT ---
  Wire.begin(PN532_SDA, PN532_SCL);
  Wire.setTimeOut(1000); // Prevent hangs

  pn532.begin();
  if (!pn532.getFirmwareVersion()) {
    tft.fillScreen(COLOR_RED);
    tft.setCursor(0,0); tft.print("NFC ERROR");
    while (1) delay(100);
  }
  pn532.SAMConfig();

  if (!rtc.begin()) Serial.println("RTC ERROR");
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // --- SERVO INIT ---
  doorServo.attach(SERVO_PIN, 500, 2400);
  doorServo.write(0);
  
  // --- FPGA PIN INIT ---
  pinMode(BUS_D0, OUTPUT); pinMode(BUS_D1, OUTPUT);
  pinMode(BUS_D2, OUTPUT); pinMode(BUS_D3, OUTPUT);
  pinMode(BUS_D4, OUTPUT); pinMode(BUS_D5, OUTPUT);
  pinMode(BUS_D6, OUTPUT); pinMode(BUS_D7, OUTPUT);
  pinMode(BUS_CLK, OUTPUT);
  pinMode(BUS_LATCH, OUTPUT);
  pinMode(FPGA_VALID, INPUT);
  pinMode(FPGA_REPLAY, INPUT);

  // Initial Screen Draw
  resetIdleScreen();
}

// =================== 8. MAIN LOOP ===================

void loop() {
  // 1. Pet Watchdog (Prevent reboot)
  esp_task_wdt_reset();

  // 2. Serial Command Listener ("LOGS")
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); 
    if (cmd == "LOGS") printLogs();
  }

  // 3. Update Clock (Every 1000ms)
  if (millis() - lastClockUpdate > 1000) {
    if (!needsScreenReset) {
      drawClock();
    }
    lastClockUpdate = millis();
  }

  // 4. NFC Logic
  uint8_t uid[7];
  uint8_t uidLen;

  // Short timeout (50ms) to keep loop running fast
  if (pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 50)) {
    String tag = "";
    for (uint8_t i = 0; i < uidLen; i++) {
      if (uid[i] < 0x10) tag += "0";
      tag += String(uid[i], HEX);
    }
    tag.toUpperCase();
    
    updateStatus("PROCESSING", tag.c_str(), COLOR_YELLOW);

    // Check Whitelist
    if (isAllowed(tag)) {
      authCounter++;
      uint64_t token = generateToken(tag);
      sendTokenToFPGA(token);
      delay(20); // Wait for FPGA processing

      bool v = digitalRead(FPGA_VALID);
      bool r = digitalRead(FPGA_REPLAY);

      if (v && !r) {
        // --- ACCESS GRANTED ---
        logAccess(tag);
        updateStatus("ACCESS", "GRANTED", COLOR_GREEN);
        doorServo.write(90);  // Open
        delay(2000);
        doorServo.write(0);   // Close
      } else if (r) {
        // --- REPLAY ATTACK ---
        updateStatus("DENIED", "REPLAY!", COLOR_RED);
        delay(2000);
      } else {
        // --- INVALID TOKEN ---
        updateStatus("DENIED", "INVALID", COLOR_RED);
        delay(2000);
      }
    } else {
      // --- UNKNOWN CARD ---
      updateStatus("DENIED", "UNKNOWN", COLOR_RED);
      delay(2000);
    }
    
    // Return to "Ready" screen
    resetIdleScreen();
  }
}
