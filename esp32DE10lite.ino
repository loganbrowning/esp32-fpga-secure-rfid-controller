#include <Wire.h>
#include <Adafruit_PN532.h>
#include <ESP32Servo.h>
#include <LiquidCrystal.h>

// =================== PIN DEFINITIONS (MATCHES YOUR WIRING) ===================

// PN532 in I2C mode (bit-banged using these pins)
#define PN532_SDA 21
#define PN532_SCL 22

// Servo
#define SERVO_PIN 17

// LCD (16x2, 4-bit parallel)
// RS, E, D4, D5, D6, D7
// D7 = GPIO2 exactly as you wired it
LiquidCrystal lcd(19, 18, 23, 22, 21, 2);

// FPGA Parallel Bus (ESP32 → DE10-Lite)
#define BUS_D0 25
#define BUS_D1 26
#define BUS_D2 27
#define BUS_D3 32
#define BUS_D4 33
#define BUS_D5 12
#define BUS_D6 13
#define BUS_D7 14

// IMPORTANT: these reflect your *actual* wiring
#define BUS_CLK   4   // JP1 pin 9 = GPIO[8] = bus_clk
#define BUS_LATCH 5   // JP1 pin 10 = GPIO[9] = bus_latch

// FPGA status (DE10-Lite → ESP32)
#define FPGA_VALID  34   // JP1 pin 13 (GPIO[10])
#define FPGA_REPLAY 35   // JP1 pin 14 (GPIO[11])

// Optional red LED on GPIO15 (you can leave unwired if you want)
#define LED_RED 15

// PN532 object in I2C mode using your SDA/SCL
Adafruit_PN532 pn532(PN532_SDA, PN532_SCL);

// Servo object
Servo doorServo;

// =================== ACCESS CONTROL CONFIG ===================

// Your real card UIDs (from your tests), uppercase hex, no "0x"
String allowedTags[] = {
  "B5F84306",
  "9C27F905"
};
int numTags = sizeof(allowedTags) / sizeof(allowedTags[0]);

// =================== HELPERS ===================

bool isAllowed(const String &tag) {
  for (int i = 0; i < numTags; i++) {
    if (tag == allowedTags[i]) return true;
  }
  return false;
}

// Hash UID string into 64-bit token (advanced: “identity” of a card)
uint64_t generateTokenFromUID(const String &tag) {
  uint64_t h = 0;
  for (size_t i = 0; i < tag.length(); i++) {
    h = (h << 5) + (uint8_t)tag[i];  // simple rolling hash
  }
  return h;
}

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
  // Send MSB-first: 8 bytes
  for (int i = 7; i >= 0; i--) {
    uint8_t b = (token >> (i * 8)) & 0xFF;
    setBusByte(b);

    digitalWrite(BUS_CLK, HIGH);
    delayMicroseconds(5);
    digitalWrite(BUS_CLK, LOW);
    delayMicroseconds(5);
  }

  // Latch rising edge = "token ready"
  digitalWrite(BUS_LATCH, HIGH);
  delayMicroseconds(10);
  digitalWrite(BUS_LATCH, LOW);
}

void unlockDoor() {
  lcd.clear();
  lcd.print("ACCESS GRANTED");
  lcd.setCursor(0, 1);
  lcd.print("Unlocking...");

  doorServo.write(90);   // open
  delay(1500);
  doorServo.write(0);    // close

  lcd.clear();
  lcd.print("Tap card...");
}

void denyAccess(bool replay) {
  digitalWrite(LED_RED, HIGH);

  lcd.clear();
  lcd.print("ACCESS DENIED");
  lcd.setCursor(0, 1);
  if (replay) {
    lcd.print("REPLAY ATTACK");
  } else {
    lcd.print("INVALID CARD");
  }

  delay(2000);
  digitalWrite(LED_RED, LOW);

  lcd.clear();
  lcd.print("Tap card...");
}

void setupBusPins() {
  pinMode(BUS_D0, OUTPUT);
  pinMode(BUS_D1, OUTPUT);
  pinMode(BUS_D2, OUTPUT);
  pinMode(BUS_D3, OUTPUT);
  pinMode(BUS_D4, OUTPUT);
  pinMode(BUS_D5, OUTPUT);
  pinMode(BUS_D6, OUTPUT);
  pinMode(BUS_D7, OUTPUT);

  pinMode(BUS_CLK, OUTPUT);
  pinMode(BUS_LATCH, OUTPUT);

  digitalWrite(BUS_CLK, LOW);
  digitalWrite(BUS_LATCH, LOW);

  pinMode(FPGA_VALID, INPUT);
  pinMode(FPGA_REPLAY, INPUT);
}

// =================== SETUP ===================

void setup() {
  Serial.begin(115200);
  delay(300);

  // LCD
  lcd.begin(16, 2);
  lcd.print("Booting PN532...");

  // PN532
  Wire.begin();
  pn532.begin();

  uint32_t ver = pn532.getFirmwareVersion();
  if (!ver) {
    lcd.clear();
    lcd.print("NO PN532 FOUND");
    Serial.println("PN532 NOT FOUND!");
    while (1) {
      delay(100);
    }
  }

  pn532.SAMConfig();  // enable reader mode

  // Servo
  doorServo.attach(SERVO_PIN, 500, 2400);
  doorServo.write(0);

  // LED
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);

  // FPGA bus
  setupBusPins();

  lcd.clear();
  lcd.print("Tap card...");
  Serial.println("System ready. Tap a card.");
}

// =================== MAIN LOOP ===================

void loop() {
  uint8_t uid[7];
  uint8_t uidLen;

  // Non-blocking PN532 read
  if (pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 0)) {

    // Build UID string
    String tag = "";
    for (uint8_t i = 0; i < uidLen; i++) {
      if (uid[i] < 0x10) tag += "0";
      tag += String(uid[i], HEX);
    }
    tag.toUpperCase();

    Serial.print("Card detected! UID = ");
    Serial.println(tag);

    lcd.clear();
    lcd.print("Detected:");
    lcd.setCursor(0, 1);
    lcd.print(tag.substring(0, 16));

    // Check if this card is allowed
    if (!isAllowed(tag)) {
      denyAccess(false);
      delay(500);
      return;
    }

    // Generate 64-bit token from UID
    uint64_t token = generateTokenFromUID(tag);

    Serial.print("Sending token upper32: 0x");
    Serial.println((uint32_t)(token >> 32), HEX);

    // Send token to FPGA
    sendTokenToFPGA(token);
    delay(20);  // give FPGA time to process

    bool v = digitalRead(FPGA_VALID);
    bool r = digitalRead(FPGA_REPLAY);

    Serial.print("FPGA_VALID=");
    Serial.print(v);
    Serial.print(" FPGA_REPLAY=");
    Serial.println(r);

    if (v && !r) {
      unlockDoor();
    } else if (r) {
      denyAccess(true);  // replay
    } else {
      // Neither set → treat as generic deny
      denyAccess(false);
    }

    delay(500); // avoid re-reading card continuously
  }
}
