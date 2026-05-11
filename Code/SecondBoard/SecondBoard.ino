#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

#define SS_PIN     10
#define RST_PIN    8
#define BUZZER_PIN 6

MFRC522 rfid(SS_PIN, RST_PIN);
DFRobotDFPlayerMini dfPlayer;

SoftwareSerial mySerial(4, 5); // RX=D4, TX=D5

byte card1[4] = {0x13, 0xF8, 0x33, 0x02};
byte card2[4] = {0x83, 0x0C, 0x46, 0xDA};

unsigned long lastScanTime = 0;
bool systemOn = false;

// ===================== BUZZER =====================
int buzzerMode = 0; // 0=off 1=slow 2=med 3=fast
unsigned long lastBeepTime = 0;
bool buzzerState = false;

void updateBuzzer() {
  if (buzzerMode == 0) {
    noTone(BUZZER_PIN);
    return;
  }

  int freq, onMs, offMs;
  if      (buzzerMode == 1) { freq = 1400;  onMs = 250; offMs = 250; } // 75–150 سم
  else if (buzzerMode == 2) { freq = 1400; onMs = 100; offMs = 100; } // 25–75 سم
  else                      { freq = 1400; onMs = 30;  offMs = 30;  } // أقل من 25 سم

  unsigned long interval = buzzerState ? onMs : offMs;
  if (millis() - lastBeepTime >= interval) {
    lastBeepTime = millis();
    buzzerState  = !buzzerState;
    if (buzzerState) tone(BUZZER_PIN, freq);
    else             noTone(BUZZER_PIN);
  }
}

// ===================== RFID =====================
bool checkCard(const byte *uid) {
  for (byte i = 0; i < 4; i++) {
    if (rfid.uid.uidByte[i] != uid[i]) return false;
  }
  return true;
}

void preBeep() {
  tone(BUZZER_PIN, 800);
  delay(120);
  tone(BUZZER_PIN, 1000);
  delay(120);
  tone(BUZZER_PIN, 1200);
  delay(120);
  noTone(BUZZER_PIN);
}

// ===================== SETUP =====================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);

  SPI.begin();
  rfid.PCD_Init();

  Serial.begin(9600);
  delay(1200);

  if (!dfPlayer.begin(Serial)) {
    while (true) {
      tone(BUZZER_PIN, 400);
      delay(150);
      noTone(BUZZER_PIN);
      delay(150);
    }
  }

  dfPlayer.volume(30);
  dfPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  dfPlayer.EQ(DFPLAYER_EQ_NORMAL);

  mySerial.begin(9600);
  mySerial.println("READY");
}

// ===================== LOOP =====================
void loop() {
  updateBuzzer();
  // RFID scan
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    if (millis() - lastScanTime > 2000) {
      if (checkCard(card1) || checkCard(card2)) {
        systemOn = !systemOn;
        preBeep();

        if (systemOn) {
          mySerial.println("ON");
        } else {
          mySerial.println("OFF");
          buzzerMode  = 0;
          buzzerState = false;
          noTone(BUZZER_PIN);
        }

        lastScanTime = millis();
      }
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }

  // Commands from main board
  if (mySerial.available()) {
    char cmd[16];
    memset(cmd, 0, sizeof(cmd));
    mySerial.readBytesUntil('\n', cmd, sizeof(cmd) - 1);
    int len = strlen(cmd);
    if (len > 0 && cmd[len - 1] == '\r') cmd[len - 1] = '\0';

    if (strncmp(cmd, "PLAY_", 5) == 0) {
      int track = atoi(cmd + 5);
      if (track > 0) dfPlayer.play(track);
    }
    else if (strcmp(cmd, "STOP_AUDIO") == 0) { dfPlayer.stop(); noTone(BUZZER_PIN); }
    else if (strcmp(cmd, "B_FAST") == 0)     { buzzerMode = 3; }
    else if (strcmp(cmd, "B_MED") == 0)      { buzzerMode = 2; }
    else if (strcmp(cmd, "B_SLOW") == 0)     { buzzerMode = 1; }
    else if (strcmp(cmd, "B_OFF") == 0)      { buzzerMode = 0; buzzerState = false; noTone(BUZZER_PIN); }
  }
}
