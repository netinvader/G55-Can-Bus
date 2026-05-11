#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <SPI.h>
#include <FlexCAN_T4.h>
#include <math.h>

// =====================================================
// HARDWARE
// =====================================================
// Teensy 4.1 CAN1:
// CTX1 = pin 22
// CRX1 = pin 23
//
// GC9A01 round display:
// SCK  = pin 13
// MOSI = pin 11
// CS   = pin 10
// DC   = pin 9
// RST  = pin 8
// VCC  = 3.3V
// GND  = GND
// BL   = 3.3V
// =====================================================

#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8

Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// =====================================================
// COLORS
// =====================================================
#define BLACK       0x0000
#define WHITE       0xFFFF
#define HOT_PINK    0xF81F
#define NEON_GREEN  0x7FE0
#define DIM_PURPLE  0x3008
#define SOFT_WHITE  0xE71C
#define DARK_GRAY   0x18E3
#define RED         0xF800
#define YELLOW      0xFFE0

// =====================================================
// OBD CONSTANTS
// =====================================================
#define OBD_FUNCTIONAL_REQUEST_ID 0x7DF

// Safe Mode 01 allowlist only.
enum ObdPid : uint8_t {
  PID_COOLANT_TEMP = 0x05,
  PID_MAP          = 0x0B,
  PID_RPM          = 0x0C,
  PID_SPEED        = 0x0D,
  PID_IAT          = 0x0F,
  PID_MAF          = 0x10,
  PID_THROTTLE     = 0x11,
  PID_FUEL_LEVEL   = 0x2F,
  PID_MODULE_VOLT  = 0x42
};

const uint8_t pollPids[] = {
  PID_RPM,
  PID_SPEED,
  PID_COOLANT_TEMP,
  PID_THROTTLE,
  PID_IAT,
  PID_MAP
};

const int pollPidCount = sizeof(pollPids) / sizeof(pollPids[0]);

// =====================================================
// LIVE DATA
// =====================================================
uint32_t rxCount = 0;
uint32_t txCount = 0;
uint32_t errCount = 0;

uint32_t lastRxMillis = 0;
uint32_t lastPollMillis = 0;
uint32_t lastDisplayMillis = 0;

int pollIndex = 0;

bool rpmValid = false;
bool speedValid = false;
bool coolantValid = false;
bool throttleValid = false;
bool iatValid = false;
bool mapValid = false;

int rpm = 0;
int mph = 0;
int coolantF = 0;
int iatF = 0;
int mapKpa = 0;
float throttlePct = 0.0;

uint32_t lastId = 0;
uint8_t lastLen = 0;
uint8_t lastData[8] = {0};

// =====================================================
// UTILS
// =====================================================
bool isAllowedPid(uint8_t pid) {
  for (int i = 0; i < pollPidCount; i++) {
    if (pollPids[i] == pid) return true;
  }
  return false;
}

int cToF(int c) {
  return (int)round((c * 9.0 / 5.0) + 32.0);
}

// =====================================================
// CAN TX: SAFE OBD MODE 01 ONLY
// =====================================================
void sendObdRequest(uint8_t pid) {
  if (!isAllowedPid(pid)) {
    errCount++;
    return;
  }

  CAN_message_t msg;
  msg.id = OBD_FUNCTIONAL_REQUEST_ID;
  msg.flags.extended = 0;
  msg.len = 8;

  msg.buf[0] = 0x02;  // 2 data bytes follow
  msg.buf[1] = 0x01;  // Mode 01 current data
  msg.buf[2] = pid;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;

  if (Can1.write(msg)) {
    txCount++;
  } else {
    errCount++;
  }
}

// =====================================================
// CAN RX: DECODE MODE 01 RESPONSES
// Expected response:
// ID 0x7E8-0x7EF
// buf[0] = length
// buf[1] = 0x41
// buf[2] = PID
// =====================================================
void decodeObdResponse(const CAN_message_t &msg) {
  if (msg.id < 0x7E8 || msg.id > 0x7EF) return;
  if (msg.len < 4) return;
  if (msg.buf[1] != 0x41) return;

  uint8_t pid = msg.buf[2];

  switch (pid) {
    case PID_RPM: {
      if (msg.len >= 5) {
        int raw = ((int)msg.buf[3] << 8) | msg.buf[4];
        rpm = raw / 4;
        rpmValid = true;
      }
      break;
    }

    case PID_SPEED: {
      mph = (int)round(msg.buf[3] * 0.621371);
      speedValid = true;
      break;
    }

    case PID_COOLANT_TEMP: {
      int c = (int)msg.buf[3] - 40;
      coolantF = cToF(c);
      coolantValid = true;
      break;
    }

    case PID_THROTTLE: {
      throttlePct = msg.buf[3] * 100.0 / 255.0;
      throttleValid = true;
      break;
    }

    case PID_IAT: {
      int c = (int)msg.buf[3] - 40;
      iatF = cToF(c);
      iatValid = true;
      break;
    }

    case PID_MAP: {
      mapKpa = msg.buf[3];
      mapValid = true;
      break;
    }

    default:
      break;
  }
}

void readCAN() {
  CAN_message_t msg;

  while (Can1.read(msg)) {
    rxCount++;
    lastRxMillis = millis();

    lastId = msg.id;
    lastLen = msg.len;

    for (int i = 0; i < 8; i++) {
      lastData[i] = (i < msg.len) ? msg.buf[i] : 0;
    }

    decodeObdResponse(msg);

    Serial.print("RX ID 0x");
    Serial.print(msg.id, HEX);
    Serial.print(" DLC ");
    Serial.print(msg.len);
    Serial.print(" DATA ");

    for (int i = 0; i < msg.len; i++) {
      if (msg.buf[i] < 0x10) Serial.print("0");
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
}

// =====================================================
// DISPLAY DRAWING
// =====================================================
void drawArc(int cx, int cy, int r, int startDeg, int endDeg, uint16_t color, int thickness) {
  for (int t = 0; t < thickness; t++) {
    int rr = r - t;

    for (int a = startDeg; a <= endDeg; a += 2) {
      float rad1 = a * DEG_TO_RAD;
      float rad2 = (a + 2) * DEG_TO_RAD;

      int x1 = cx + cos(rad1) * rr;
      int y1 = cy + sin(rad1) * rr;
      int x2 = cx + cos(rad2) * rr;
      int y2 = cy + sin(rad2) * rr;

      tft.drawLine(x1, y1, x2, y2, color);
    }
  }
}

void drawTicks() {
  for (int a = 150; a <= 210; a += 8) {
    float rad = a * DEG_TO_RAD;
    int x1 = 120 + cos(rad) * 103;
    int y1 = 120 + sin(rad) * 103;
    int x2 = 120 + cos(rad) * 111;
    int y2 = 120 + sin(rad) * 111;
    tft.drawLine(x1, y1, x2, y2, WHITE);
  }

  for (int a = 330; a <= 390; a += 8) {
    float rad = a * DEG_TO_RAD;
    int x1 = 120 + cos(rad) * 103;
    int y1 = 120 + sin(rad) * 103;
    int x2 = 120 + cos(rad) * 111;
    int y2 = 120 + sin(rad) * 111;
    tft.drawLine(x1, y1, x2, y2, WHITE);
  }
}

void drawBackgroundTexture() {
  for (int y = 65; y < 130; y += 12) {
    for (int x = 35; x < 205; x += 14) {
      tft.drawPixel(x, y, DIM_PURPLE);
      tft.drawPixel(x + 1, y + 1, DIM_PURPLE);
      tft.drawPixel(x + 2, y, DIM_PURPLE);
      tft.drawPixel(x + 1, y - 1, DIM_PURPLE);
    }
  }
}

void drawStaticFrame() {
  tft.fillScreen(BLACK);

  drawArc(120, 120, 112, 210, 330, HOT_PINK, 3);
  drawArc(120, 120, 112, 30, 150, HOT_PINK, 3);

  drawArc(120, 120, 98, 205, 335, HOT_PINK, 1);
  drawArc(120, 120, 98, 25, 155, HOT_PINK, 1);

  drawTicks();
  drawBackgroundTexture();

  tft.setTextColor(SOFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(58, 22);
  tft.print("G55 CAN");

  tft.drawLine(35, 72, 52, 60, HOT_PINK);
  tft.drawLine(52, 60, 188, 60, HOT_PINK);
  tft.drawLine(188, 60, 205, 72, HOT_PINK);

  tft.drawLine(38, 151, 202, 151, HOT_PINK);
  tft.drawLine(35, 158, 205, 158, HOT_PINK);

  tft.drawLine(86, 164, 86, 205, HOT_PINK);
  tft.drawLine(155, 164, 155, 205, HOT_PINK);

  tft.drawLine(40, 196, 87, 196, HOT_PINK);
  tft.drawLine(153, 196, 200, 196, HOT_PINK);
}

void printValueOrDashInt(bool valid, int value) {
  if (valid) {
    tft.print(value);
  } else {
    tft.print("--");
  }
}

void drawLiveScreen() {
  bool online = (millis() - lastRxMillis) < 1500 && rxCount > 0;

  drawStaticFrame();

  if (online) {
    tft.fillCircle(54, 51, 5, NEON_GREEN);
    tft.setTextColor(NEON_GREEN);
    tft.setTextSize(2);
    tft.setCursor(70, 44);
    tft.print("OBD OK");
  } else {
    tft.fillCircle(54, 51, 5, RED);
    tft.setTextColor(RED);
    tft.setTextSize(2);
    tft.setCursor(70, 44);
    tft.print("WAIT");
  }

  // Central RPM
  tft.setTextColor(HOT_PINK);
  tft.setTextSize(2);
  tft.setCursor(96, 77);
  tft.print("RPM");

  tft.setTextColor(NEON_GREEN);
  tft.setTextSize(5);
  tft.setCursor(50, 101);

  if (rpmValid) {
    tft.print(rpm);
  } else {
    tft.print("----");
  }

  // Lower row labels
  tft.setTextSize(2);

  tft.setTextColor(HOT_PINK);
  tft.setCursor(39, 164);
  tft.print("MPH");

  tft.setTextColor(WHITE);
  tft.setCursor(43, 187);
  printValueOrDashInt(speedValid, mph);

  tft.setTextColor(HOT_PINK);
  tft.setCursor(103, 164);
  tft.print("ECT");

  tft.setTextColor(WHITE);
  tft.setCursor(98, 187);
  printValueOrDashInt(coolantValid, coolantF);
  if (coolantValid) tft.print("F");

  tft.setTextColor(HOT_PINK);
  tft.setCursor(174, 164);
  tft.print("TPS");

  tft.setTextColor(WHITE);
  tft.setCursor(176, 187);
  if (throttleValid) {
    tft.print((int)round(throttlePct));
    tft.print("%");
  } else {
    tft.print("--");
  }

  // Bottom status
  tft.setTextSize(1);
  tft.setTextColor(NEON_GREEN);
  tft.setCursor(49, 214);
  tft.print("IAT ");
  printValueOrDashInt(iatValid, iatF);
  if (iatValid) tft.print("F");

  tft.setCursor(122, 214);
  tft.print("MAP ");
  printValueOrDashInt(mapValid, mapKpa);
  if (mapValid) tft.print("kPa");

  tft.setTextColor(SOFT_WHITE);
  tft.setCursor(74, 226);
  tft.print("TX ");
  tft.print(txCount);
  tft.print(" RX ");
  tft.print(rxCount);
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  tft.begin();
  tft.setRotation(0);

  drawStaticFrame();
  tft.setTextColor(NEON_GREEN);
  tft.setTextSize(2);
  tft.setCursor(45, 105);
  tft.print("BOOTING");

  Can1.begin();
  Can1.setBaudRate(500000);

  Serial.println("G55 CAN Gateway");
  Serial.println("Safe OBD Mode 01 polling only");
  Serial.println("CAN1 500 kbps");
  Serial.println("TX 0x7DF, RX 0x7E8-0x7EF");

  delay(750);
  drawLiveScreen();
}

void loop() {
  readCAN();

  // Slow, rotating OBD polling.
  // One request every 300 ms.
  if (millis() - lastPollMillis >= 300) {
    lastPollMillis = millis();

    uint8_t pid = pollPids[pollIndex];
    sendObdRequest(pid);

    pollIndex++;
    if (pollIndex >= pollPidCount) pollIndex = 0;
  }

  // Refresh display 5x/sec.
  if (millis() - lastDisplayMillis >= 200) {
    lastDisplayMillis = millis();
    drawLiveScreen();
  }
}
