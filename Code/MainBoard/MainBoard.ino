#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <avr/pgmspace.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial mySerial(13, 12);

#define MPU        0x68
#define TRIG_PIN   10
#define ECHO_PIN   11
#define BUTTON_PIN A2

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {2, 3, 4, 5};
byte colPins[COLS] = {6, 7, 8, 9};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ===================== SYSTEM STATE =====================
bool systemOn = false;

// ===================== STEP COUNTER =====================
int stepCount = 0;
unsigned long lastStepTime = 0;
bool aboveThreshold = false;
int stepMode = 2; // default: gyroscope

bool lastButtonState = HIGH;
unsigned long lastButtonTime = 0;

// ===================== ULTRASONIC =====================
long duration;
int distance;
int lastBuzzerCmd = -1;

// ===================== NAVIGATION =====================
int menuState = 0;
int selectedFloor = 0;
int selectedType  = 0;
int navState = 0;
bool isSelectingLocation = false; 

int currentLocation = 0;
int pendingDestination = -1;

// ===================== ROUTE =====================
enum NavAction : uint8_t {
  ACT_FORWARD = 0,
  ACT_RIGHT   = 1,
  ACT_LEFT    = 2,
  ACT_ARRIVED = 3,
  ACT_STAIR   = 4,
  ACT_TURN    = 5
};

struct Step {
  uint8_t action;
  int steps;
};

Step navPath[16];
int navPathLen = 0;
int navPathIndex = 0;
int navStepTarget = 0;

Step returnPath[16];
int returnPathLen = 0;

bool stairsHold = false;
bool gatePrefixUsed = false;

// ===================== AUDIO DURATIONS في PROGMEM =====================
const unsigned int trackDur[] PROGMEM = {
  0,     //  0  - placeholder
  5000,  //  01 - أهلاً بك
  2500,  //  02 - من فضلك اختر الدور
  2700,  //  03 - دور أرضي 1
  2700,  //  04 - دور أول 2
  2000,  //  05 - دور ثاني 3
  2500,  //  06 - تم اختيار الدور الأول
  2500,  //  07 - اختر نوع المكان
  2500,  //  08 - مدرجات 1
  2500,  //  09 - فصول 2
  2500,  //  10 - خدمات 3
  2500,  //  11 - تم اختيار المدرجات
  2000,  //  12 - مدرج نوح 1
  2000,  //  13 - مدرج حماد 2
  2000,  //  14 - تم اختيار الفصول
  2500,  //  15 - فصل 17
  3700,  //  16 - فصل 225 أ 2
  3500,  //  17 - فصل 225 ب 3
  2000,  //  18 - تم اختيار الخدمات
  3000,  //  19 - اختر 1 للحمام الرجالي
  3000,  //  20 - اختر 2 للحمام الحريمي
  2500,  //  21 - سوف أقوم بإرشادك
  2000,  //  22 - امشي للأمام
  1500,  //  23 - لف يمين
  1500,  //  24 - لف شمال
  1500,  //  25 - تفعيل العودة
  3000,  //  26 - اضغط رمز النجمة للعودة للقائمة السابقة
  2500,  //  27 - لقد وصلت إلى وجهتك
  2500,  //  28 - اختر وجهتك الجديدة
  3000,  //  29 - يتم إضافته قريباً
  3000,  //  30 - شكراً لاستخدام العصاية الذكية
  3000,  //  31 - اصعد السلم ثم لف لليمين
  3000,  //  32 - انزل السلم ثم لف للشمال
  2000,  //  33 - لف للخلف
  2500,  //  34 - تم اختيار المكان
  3000,  //  35 - سيبدأ التوجيه من باب الكلية
  3000,  //  36 - اختر 1 للعودة للباب الرئيسي
  3000,  //  37 - اختر 2 لاختيار وجهة جديدة
  3000,   //  38 - اختر 3 للخروج
  3000,  //  39 - اختار موقعك الحالي
  3000,  //  40 - اضغط 1 للبوابة الرئيسية
  3000   //  41 - اضغط 2 لتحديد موقعك
};

inline unsigned int getTrackDur(int idx) {
  return pgm_read_word(&trackDur[idx]);
}

// ===================== AUDIO QUEUE =====================
const byte MAX_AUDIO_QUEUE = 12;  
int audioQueue[MAX_AUDIO_QUEUE];
byte audioLen = 0;
byte audioIndex = 0;
bool audioPlaying = false;
unsigned long audioEndAt = 0;

bool waitToStartOutbound = false;

void clearAudioQueue() {
  audioLen = 0;
  audioIndex = 0;
  audioPlaying = false;
  audioEndAt = 0;
}

void queueAudio(int track) {
  if (track > 0 && audioLen < MAX_AUDIO_QUEUE)
    audioQueue[audioLen++] = track;
}

void sendPlay(int track) {
  mySerial.print(F("PLAY_"));
  mySerial.println(track);
}

void updateAudioPlayer() {
  if (audioPlaying && millis() >= audioEndAt)
    audioPlaying = false;

  if (!audioPlaying && audioIndex < audioLen) {
    int track = audioQueue[audioIndex++];
    sendPlay(track);
    audioPlaying = true;
    audioEndAt = millis() + getTrackDur(track);
  }

  if (!audioPlaying && audioIndex >= audioLen) {
    audioLen = 0;
    audioIndex = 0;
  }
}

bool audioQueueFinished() {
  return (!audioPlaying && audioIndex >= audioLen);
}

// ===================== HELPERS =====================
void showLCD(const __FlashStringHelper* line1, const __FlashStringHelper* line2 = nullptr) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  if (line2) {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
}

void showLCDStr(const __FlashStringHelper* line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void appendStep(Step* path, int& len, uint8_t action, int steps) {
  if (len < 16) {
    path[len].action = action;
    path[len].steps  = steps;
    len++;
  }
}

// ===================== LOCATION DATA =====================
struct LocationData {
  int mainLinePos;
  bool isBranch;
  int branchSteps;
  uint8_t entryAction;
};

LocationData locData[9] = {
  {0,   false, 0,  ACT_FORWARD},
  {0,   false, 0,  ACT_FORWARD},
  {70,  true,  70, ACT_RIGHT},
  {152, true,  70, ACT_RIGHT},
  {202, false, 0,  ACT_FORWARD},
  {18,  false, 0,  ACT_FORWARD},
  {45,  false, 0,  ACT_FORWARD},
  {72,  false, 0,  ACT_FORWARD},
  {83,  true,  10, ACT_LEFT},
};

// ===================== GATE PREFIX / SUFFIX =====================
void appendGateOutboundPrefix(Step* path, int& len) {
  appendStep(path, len, ACT_FORWARD, 15);
  appendStep(path, len, ACT_RIGHT,   0);
  appendStep(path, len, ACT_FORWARD, 6);
  appendStep(path, len, ACT_LEFT,    0);
  appendStep(path, len, ACT_STAIR,   0);
  appendStep(path, len, ACT_FORWARD, 6);
  appendStep(path, len, ACT_RIGHT,   0);
  appendStep(path, len, ACT_FORWARD, 10);
  appendStep(path, len, ACT_RIGHT,   0);
}

void appendGateReturnSuffix(Step* path, int& len) {
  appendStep(path, len, ACT_LEFT,    0);
  appendStep(path, len, ACT_FORWARD, 10);
  appendStep(path, len, ACT_LEFT,    0);
  appendStep(path, len, ACT_STAIR,   0);
  appendStep(path, len, ACT_RIGHT,   0);
  appendStep(path, len, ACT_FORWARD, 6);
  appendStep(path, len, ACT_LEFT,    0);
  appendStep(path, len, ACT_FORWARD, 15);
}

// ===================== BRANCH HELPERS =====================
void enterBranch(Step* path, int& len, int to, bool fromGateSide) {
  if (to == 2) {
    appendStep(path, len, fromGateSide ? ACT_RIGHT : ACT_LEFT, 0);
    appendStep(path, len, ACT_FORWARD, 70);
  } else if (to == 3) {
    appendStep(path, len, fromGateSide ? ACT_RIGHT : ACT_LEFT, 0);
    appendStep(path, len, ACT_FORWARD, 70);
  } else if (to == 8) {
    appendStep(path, len, fromGateSide ? ACT_LEFT : ACT_RIGHT, 0);
    appendStep(path, len, ACT_FORWARD, 10);
  }
}

void exitNooh(Step* path, int& len, int to) {
  appendStep(path, len, ACT_TURN, 0);
  appendStep(path, len, ACT_FORWARD, 70);
  if (to == 3 || to == 8 || to == 7 || to == 4)
    appendStep(path, len, ACT_RIGHT, 0);
  else
    appendStep(path, len, ACT_LEFT, 0);
}

void exitHammad(Step* path, int& len, int to) {
  appendStep(path, len, ACT_TURN, 0);
  appendStep(path, len, ACT_FORWARD, 70);
  if (to == 4) appendStep(path, len, ACT_RIGHT, 0);
  else         appendStep(path, len, ACT_LEFT,  0);
}

void exitMenWC(Step* path, int& len, int to) {
  appendStep(path, len, ACT_TURN, 0);
  appendStep(path, len, ACT_FORWARD, 10);
  if (to == 3 || to == 4) appendStep(path, len, ACT_LEFT, 0);
  else                    appendStep(path, len, ACT_RIGHT, 0);
}

// ===================== PATH BUILDING =====================
void buildFloorPath(Step* path, int& len, int from, int to) {
  int diff = locData[to].mainLinePos - locData[from].mainLinePos;

  if      (from == 2) exitNooh  (path, len, to);
  else if (from == 3) exitHammad(path, len, to);
  else if (from == 8) exitMenWC (path, len, to);
  else if (from == 4) appendStep(path, len, ACT_TURN, 0);
  else { if (diff < 0) appendStep(path, len, ACT_TURN, 0); }

  if (diff != 0)
    appendStep(path, len, ACT_FORWARD, (diff > 0) ? diff : -diff);

  if (locData[to].isBranch)
    enterBranch(path, len, to, (diff > 0));

  appendStep(path, len, ACT_ARRIVED, 0);
}

void buildPath(int from, int to) {
  navPathLen     = 0;
  gatePrefixUsed = false;

  if (from == 0) {
    gatePrefixUsed = true;
    appendGateOutboundPrefix(navPath, navPathLen);
    from = 1;
  }

  buildFloorPath(navPath, navPathLen, from, to);
}

void buildReturnPath() {
  returnPathLen = 0;

  if (currentLocation == 0) {
    appendStep(returnPath, returnPathLen, ACT_ARRIVED, 0);
    return;
  }

  buildFloorPath(returnPath, returnPathLen, currentLocation, 1);

  if (returnPathLen > 0 && returnPath[returnPathLen - 1].action == ACT_ARRIVED)
    returnPathLen--;

  appendGateReturnSuffix(returnPath, returnPathLen);
  appendStep(returnPath, returnPathLen, ACT_ARRIVED, 0);
}

// ===================== MENUS =====================
void promptFloorMenu(bool includeWelcome = false, bool includeNotAvailable = false) {
  showLCD(F("Select Floor"), F("1=G 2=F1 3=F2"));
  clearAudioQueue();
  if (includeWelcome)      queueAudio(1);
  if (includeNotAvailable) queueAudio(29);
  queueAudio(2); queueAudio(3); queueAudio(4); queueAudio(5);
}

void promptTypeMenu(bool includeConfirm = false) {
  showLCD(F("Select Type"), F("1=Hall 2=Cls 3=Srv"));
  clearAudioQueue();
  if (includeConfirm) queueAudio(6);
  queueAudio(7); queueAudio(8); queueAudio(9); queueAudio(10);
}

void promptHallMenu() {
  clearAudioQueue();
  queueAudio(11);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("Select Hall"));
  lcd.setCursor(0, 1);
  if (currentLocation != 2) { lcd.print(F("1=Nooh ")); queueAudio(12); }
  if (currentLocation != 3) { lcd.print(F("2=Hammad"));  queueAudio(13); }
}

void promptClassMenu() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("Select Class"));
  lcd.setCursor(0, 1);
  if (currentLocation != 4) lcd.print(F("1=Cls7 "));
  if (currentLocation != 5) lcd.print(F("2=225A "));
  if (currentLocation != 6) lcd.print(F("3=225B"));
  clearAudioQueue();
  queueAudio(14); queueAudio(15); queueAudio(16); queueAudio(17);
}

void promptServiceMenu() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("Select Service"));
  lcd.setCursor(0, 1);
  if (currentLocation != 8) lcd.print(F("1=MWC "));
  if (currentLocation != 7) lcd.print(F("2=FWC"));
  clearAudioQueue();
  queueAudio(18);
  if (currentLocation != 8) queueAudio(19);
  if (currentLocation != 7) queueAudio(20);
}

void promptReturnMenu() {
  showLCD(F("1=Gate 2=Menu"), F("3=Exit"));
  menuState = 4;
  clearAudioQueue();
  queueAudio(25);
}

// ===================== RESTORE SCREEN =====================
void restoreCurrentScreen() {
  if (navState != 0) {
    Step* path = (navState == 1) ? navPath : returnPath;
    showLCDStr(F("Steps:"), String(stepCount) + "/" + String(path[navPathIndex].steps));
  } else if (menuState == 0) {
    showLCD(F("System ON"), F("Press D to nav"));
  } else if (menuState == 1) {
    showLCD(F("Select Floor"), F("1=G 2=F1 3=F2"));
  } else if (menuState == 2) {
    showLCD(F("Select Type"), F("1=Hall 2=Cls 3=Srv"));
  } else if (menuState == 3) {
    if      (selectedType == 1) showLCD(F("Select Hall"), F("1=Nooh 2=Hammad"));
    else if (selectedType == 2) showLCD(F("Select Class"), F("1=Cls7 2=225A 3=225B"));
    else if (selectedType == 3) showLCD(F("Select Service"), F("1=MWC 2=FWC"));
  } else if (menuState == 4) {
    showLCD(F("1=Gate 2=Menu"), F("3=Exit"));
  }
}


void playDirectionForAction(uint8_t action) {
  if      (action == ACT_FORWARD) queueAudio(22);
  else if (action == ACT_RIGHT)   queueAudio(23);
  else if (action == ACT_LEFT)    queueAudio(24);
  else if (action == ACT_TURN)    queueAudio(33);
}


void handleArrival();

void beginCurrentSegment() {
  Step* path = (navState == 1) ? navPath : returnPath;
  int   len  = (navState == 1) ? navPathLen : returnPathLen;

  while (navPathIndex < len) {
    Step s = path[navPathIndex];

    if (s.action == ACT_ARRIVED) { handleArrival(); return; }

    if (s.action == ACT_STAIR) {
      stairsHold    = true;
      stepCount     = 0;
      navStepTarget = 0;
      if (navState == 1) {
        showLCD(F("Stairs"), F("Press btn to up"));
        queueAudio(31);
      } else {
        showLCD(F("Stairs"), F("Press btn to down"));
        queueAudio(32);
      }
      return;
    }

    if (s.action == ACT_TURN) {
      showLCD(F("Turn Back"), nullptr);
      queueAudio(33);
      navPathIndex++;
      continue;
    }

    if (s.steps == 0) {
      if (s.action == ACT_RIGHT) { showLCD(F("Turn Right"), nullptr); queueAudio(23); }
      else if (s.action == ACT_LEFT) { showLCD(F("Turn Left"), nullptr); queueAudio(24); }
      navPathIndex++;
      continue;
    }

    stairsHold    = false;
    stepCount     = 0;
    navStepTarget = s.steps;

    showLCDStr(F("Steps:"), String(stepCount) + "/" + String(navStepTarget));
    if (s.action == ACT_FORWARD) { queueAudio(22); return; }
    playDirectionForAction(s.action);
    return;
  }

  handleArrival();
}

void resumeFromStairs() {
  stairsHold = false;
  stepCount  = 0;
  if (navState == 1 && gatePrefixUsed) currentLocation = 1;
  navPathIndex++;
  beginCurrentSegment();
}

// ===================== STEP HANDLING =====================
void handleStepButton() {
  bool currentState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && currentState == LOW) {
    if (millis() - lastButtonTime > 300) {
      lastButtonTime = millis();
      if (navState != 0 && stairsHold) {
        resumeFromStairs();
      } else if (stepMode == 1 && navState != 0) {
        stepCount++;
      }
    }
  }
  lastButtonState = currentState;
}

void updateStepGyro() {
  if (navState == 0 || stairsHold) return;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  float x   = ax / 16384.0;
  float y   = ay / 16384.0;
  float z   = az / 16384.0;
  float mag = sqrt(x*x + y*y + z*z);

  if (!aboveThreshold && mag > 1.05) aboveThreshold = true;

  if (aboveThreshold && mag < 1.1) {
    if (millis() - lastStepTime > 400) {
      stepCount++;
      lastStepTime = millis();
    }
    aboveThreshold = false;
  }
}

// ===================== NAVIGATION CONTROL =====================
void startReturnToGate() {
  buildReturnPath();
  navState           = 2;
  navPathIndex       = 0;
  stepCount          = 0;
  navStepTarget      = 0;
  stairsHold         = false;
  pendingDestination = 0;
  menuState          = 0;
  clearAudioQueue();
  showLCD(F("Returning"), F("Main gate"));
  beginCurrentSegment();
}

void startNavigation(int destination) {
  pendingDestination = destination;
  navPathIndex       = 0;
  stepCount          = 0;
  navStepTarget      = 0;
  stairsHold         = false;
  navState           = 1;
  menuState          = 0;
  clearAudioQueue();
  showLCD(F("Navigation"), F("Starting..."));
  buildPath(currentLocation, destination);
  queueAudio(34);
  queueAudio(21);
  if (currentLocation == 0) queueAudio(35);
  waitToStartOutbound = true;
}

void handleArrival() {
  navState      = 0;
  stairsHold    = false;
  stepCount     = 0;
  navStepTarget = 0;
  waitToStartOutbound = false;

  if (pendingDestination != -1) {
    currentLocation    = pendingDestination;
    pendingDestination = -1;
  }

  clearAudioQueue();

  if (currentLocation == 0) {
    menuState = 0;
    showLCD(F("Arrived at Gate"), F("Press D to nav"));
    queueAudio(27);
  } else {
    menuState = 4;
    showLCD(F("1=Gate 2=Menu"), F("3=Exit"));
    queueAudio(27); queueAudio(28);
  }
}

void resetNavigationState() {
  navState = 0;
  menuState = 0;
  stepCount = 0;
  navStepTarget = 0;
  stairsHold = false;
  gatePrefixUsed = false;
  pendingDestination = -1;
  waitToStartOutbound = false;
  navPathLen = 0;
  navPathIndex = 0;
  returnPathLen = 0;
  clearAudioQueue();
}

void cancelNavigation() {
  mySerial.println(F("STOP_AUDIO"));
  resetNavigationState();
  currentLocation = 0;
  isSelectingLocation = false;
  showLCD(F("System ON"), F("Press D to nav"));
}

void exitToIdle() {
  mySerial.println(F("STOP_AUDIO"));
  resetNavigationState();
  currentLocation = 0;
  isSelectingLocation = false;
  queueAudio(30);
  showLCD(F("System ON"), F("Press D to nav"));
}

void promptLocationMenu() {
  showLCD(F("Your location?"), F("1=Gate  2=Menu"));
  menuState = 5;
  clearAudioQueue();
  queueAudio(1);   // أهلاً بك
  queueAudio(26);  // اضغط * للرجوع
  queueAudio(39);  // اختار موقعك الحالي
  queueAudio(40);  // اضغط 1 للبوابة الرئيسية
  queueAudio(41);  // اضغط 2 لتحديد موقعك
}

// ===================== MENU HANDLER =====================
void handleMenu(char key) {

  if (menuState == 5) {
    if (key == '1') {
      currentLocation = 0;
      isSelectingLocation = false;
      menuState = 1;
      promptFloorMenu(false);
    } else if (key == '2') {
      isSelectingLocation = true;
      selectedFloor = 0;
      selectedType  = 0;
      menuState = 1;
      promptFloorMenu(false);
    }
    return;
  }

  if (menuState == 0 && key == 'D') {
    promptLocationMenu();
    return;
  }

  if (menuState == 4) {
    if      (key == '1') startReturnToGate();
    else if (key == '2') {
      isSelectingLocation = false;
      selectedFloor = 0;
      selectedType  = 0;
      menuState = 1;
      promptFloorMenu(false);
    }
    else if (key == '3') exitToIdle();
    else if (key == '*') { menuState = 0; showLCD(F("System ON"), F("Press D to nav")); clearAudioQueue(); }
    return;
  }

  if (menuState == 1) {
    if (key == '2') {
      selectedFloor = 2;
      menuState     = 2;
      promptTypeMenu(!isSelectingLocation);
    } else if (key == '1' || key == '3') {
      promptFloorMenu(false, true);
    } else if (key == '*') {
      if (isSelectingLocation) {
        menuState = 5;
        promptLocationMenu();
      } else {
        menuState = 0;
        showLCD(F("System ON"), F("Press D to nav"));
        clearAudioQueue();
      }
    }
    return;
  }

  if (menuState == 2) {
    if      (key == '1') { selectedType = 1; menuState = 3; promptHallMenu(); }
    else if (key == '2') { selectedType = 2; menuState = 3; promptClassMenu(); }
    else if (key == '3') { selectedType = 3; menuState = 3; promptServiceMenu(); }
    else if (key == '*') { menuState = 1; promptFloorMenu(false); }
    return;
  }

  if (menuState == 3) {
    if (key == '*') { menuState = 2; promptTypeMenu(false); return; }

    int selected = -1;

    if (selectedType == 1) {
      if      (key == '1') selected = 2;
      else if (key == '2') selected = 3;
      else return;
    } else if (selectedType == 2) {
      if      (key == '1') selected = 4;
      else if (key == '2') selected = 5;
      else if (key == '3') selected = 6;
      else return;
    } else if (selectedType == 3) {
      if      (key == '1') selected = 8;
      else if (key == '2') selected = 7;
      else return;
    }

    if (selected == -1) return;

    if (isSelectingLocation) {
      currentLocation    = selected;
      isSelectingLocation = false;
      selectedFloor      = 0;
      selectedType       = 0;
      menuState = 1;
      promptFloorMenu(false);
      clearAudioQueue();
      queueAudio(28);
      queueAudio(2); queueAudio(3); queueAudio(4); queueAudio(5);
    } else {
      if (selected == currentLocation) return;
      menuState = 0;
      startNavigation(selected);
    }
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);

  lcd.init();
  lcd.backlight();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  mySerial.begin(9600);
  mySerial.println(F("READY"));

  showLCD(F("System Ready"), nullptr);
}

// ===================== LOOP =====================
void loop() {
  updateAudioPlayer();

  if (waitToStartOutbound && audioQueueFinished()) {
    waitToStartOutbound = false;
    beginCurrentSegment();
  }

  char key = keypad.getKey();

  if (key) {
    Serial.print(F("Key: "));
    Serial.println(key);

    if (systemOn && key == 'C') { cancelNavigation(); return; }

    if (systemOn && key == 'A') {
      stepMode = 2;
      aboveThreshold = false;
      showLCD(F("Mode: Gyroscope"), F(""));
      delay(1500);
      restoreCurrentScreen();
      return;
    }

    if (systemOn && key == 'B') {
      stepMode = 1;
      showLCD(F("Mode: Button"), F(""));
      delay(1500);
      restoreCurrentScreen();
      return;
    }

    if (systemOn && navState == 0 && key == 'D') { handleMenu(key); return; }

    if (systemOn && (navState == 1 || navState == 2) && key == '*') { cancelNavigation(); return; }

    if (systemOn && (navState == 1 || navState == 2) && key == '#') {
      if (stairsHold) { resumeFromStairs(); return; }
      Step* path = (navState == 1) ? navPath : returnPath;
      int   len  = (navState == 1) ? navPathLen : returnPathLen;
      navPathIndex++;
      if (navPathIndex >= len || path[navPathIndex].action == ACT_ARRIVED)
        handleArrival();
      else
        beginCurrentSegment();
      return;
    }

    if (systemOn && navState == 0) handleMenu(key);
  }

  // Bluetooth from board 1
  if (mySerial.available()) {
    String cmd = mySerial.readStringUntil('\n');
    cmd.trim();

    if (cmd == F("READY")) {
      showLCD(F("System Ready"), nullptr);
    } else if (cmd == F("ON")) {
      systemOn = true;
      navState = 0; menuState = 0; stepCount = 0; stepMode = 2;
      currentLocation = 0; pendingDestination = -1;
      aboveThreshold = false; stairsHold = false;
      gatePrefixUsed = false; waitToStartOutbound = false;
      isSelectingLocation = false;
      clearAudioQueue();
      showLCD(F("System ON"), F("Powering on in 3"));
      delay(1000);
      showLCD(F("System ON"), F("Powering on in 2"));
      delay(1000);
      showLCD(F("System ON"), F("Powering on in 1"));
      delay(1000);
      showLCD(F("System ON"), F("Press D to nav"));
    } else if (cmd == F("OFF")) {
      systemOn = false;
      navState = 0; menuState = 0; stepMode = 0;
      currentLocation = 0; pendingDestination = -1;
      stairsHold = false; gatePrefixUsed = false;
      waitToStartOutbound = false;
      clearAudioQueue();
      showLCD(F("System OFF"), F("Shutting down 3"));
      delay(1000);
      showLCD(F("System OFF"), F("Shutting down 2"));
      delay(1000);
      showLCD(F("System OFF"), F("Shutting down 1"));
      delay(1000);
      showLCD(F("System OFF"), nullptr);
    }
  }

  if (systemOn) {
    handleStepButton();
    if (stepMode == 2) updateStepGyro();

    if ((navState == 1 || navState == 2) && !stairsHold) {
      Step* path = (navState == 1) ? navPath : returnPath;
      int   len  = (navState == 1) ? navPathLen : returnPathLen;

      if (navPathIndex < len && navStepTarget > 0 && stepCount >= navStepTarget) {
        navPathIndex++;
        if (navPathIndex >= len || path[navPathIndex].action == ACT_ARRIVED)
          handleArrival();
        else
          beginCurrentSegment();
      }

      if (stepMode != 0) {
        lcd.setCursor(0, 0); lcd.print(F("Steps:          "));
        lcd.setCursor(0, 1);
        lcd.print(stepCount);
        lcd.print('/');
        lcd.print(navStepTarget);
        lcd.print(F("          "));
      }
    }

    // Ultrasonic
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH, 30000);
    distance = (duration == 0) ? -1 : (int)(duration * 0.034 / 2);

    
    int newCmd;
    if      (distance == -1)  newCmd = 0;
    else if (distance <= 25)  newCmd = 3;
    else if (distance <= 75)  newCmd = 2;
    else if (distance <= 150) newCmd = 1;
    else                      newCmd = 0;

    if (newCmd != lastBuzzerCmd) {
      lastBuzzerCmd = newCmd;
      if      (newCmd == 0) mySerial.println(F("B_OFF"));
      else if (newCmd == 1) mySerial.println(F("B_SLOW"));
      else if (newCmd == 2) mySerial.println(F("B_MED"));
      else if (newCmd == 3) mySerial.println(F("B_FAST"));
      Serial.print(F("CMD: "));  
      Serial.println(newCmd);
    }
  }
}
