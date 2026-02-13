/*
 * Motion Sensing Glove - ESP32-S3 (Left glove firmware)
 *
 * Hardware:
 *  - ESP32-S3
 *  - ICM-20948 IMU over SPI (DMP Quaternion6)
 *  - 5x flex sensors to ADC pins
 *
 * BLE:
 *  - Nordic UART Service (NUS)
 *  - TX (Notify/Read):  6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 *  - RX (Write/WriteNR):6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 *
 * Commands (write to RX):
 *  - CAL   : run calibration (all fingers at once)
 *  - START : start streaming
 *  - STOP  : stop streaming
 *
 * Stream format (TX notifications):
 *  IMU,roll,pitch,yaw,S,r1,r2,r3,r4,r5,N,n1,n2,n3,n4,n5
 */

#include <Arduino.h>
#include <SPI.h>
#include "ICM_20948.h"
#include <math.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_TX_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_RX_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

static BLECharacteristic* txChar = nullptr;
static bool deviceConnected = false;

#define SERIAL_PORT              Serial
#define SERIAL_BAUDRATE          115200
#define SERIAL_USB_WAIT_MS       1500

#define PIN_SENSOR_1   4
#define PIN_SENSOR_2   5
#define PIN_SENSOR_3   6
#define PIN_SENSOR_4   7
#define PIN_SENSOR_5   8
#define ADC_RESOLUTION   12

#define CAL_PREP_MS     5000
#define CAL_HOLD_MS     5000
#define CAL_SAMPLE_MS   5

#define IMU_PIN_SCK     12
#define IMU_PIN_MOSI    11
#define IMU_PIN_MISO    13
#define IMU_PIN_CS      10

#define LOOP_DELAY_MS        20
#define ANGLE_FILTER_ALPHA   0.15f
#define FLEX_FILTER_ALPHA    0.20f

static SPIClass SPI_PORT(FSPI);
static ICM_20948_SPI myICM;

static float roll_f = 0.0f, pitch_f = 0.0f, yaw_f = 0.0f;
static float yaw_prev = 0.0f;
static float yaw_unwrapped = 0.0f;

static int fingerMin[5] = {0, 0, 0, 0, 0};
static int fingerMax[5] = {4095, 4095, 4095, 4095, 4095};
static float fingerNormF[5] = {0, 0, 0, 0, 0};

static const int fingerPins[5] = {
  PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5
};

static bool streamingEnabled = true;
static bool calibrated = false;

static inline float ema(float y, float x, float a) { return y + a * (x - y); }
static inline float clampf(float x, float lo, float hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }

static inline float map01(int x, int mn, int mx) {
  if (mx == mn) return 0.0f;
  float t = (float)(x - mn) / (float)(mx - mn);
  return clampf(t, 0.0f, 1.0f);
}

static inline float unwrapYaw(float yaw_deg) {
  float dy = yaw_deg - yaw_prev;
  if (dy > 180.0f)  dy -= 360.0f;
  if (dy < -180.0f) dy += 360.0f;
  yaw_unwrapped += dy;
  yaw_prev = yaw_deg;
  return yaw_unwrapped;
}

/**
 * Send a text line over Serial and BLE (NUS TX).
 * Splits long payloads into smaller chunks for safety.
 */
static void bleSendLine(const char* line) {
  SERIAL_PORT.println(line);

  if (!deviceConnected || txChar == nullptr) return;

  const size_t maxChunk = 180;
  size_t len = strlen(line);
  size_t i = 0;

  while (i < len) {
    size_t n = (len - i > maxChunk) ? maxChunk : (len - i);
    txChar->setValue((uint8_t*)(line + i), n);
    txChar->notify();
    i += n;
    delay(2);
  }

  txChar->setValue((uint8_t*)"\n", 1);
  txChar->notify();
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    (void)pServer;
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    delay(50);
    pServer->getAdvertising()->start();
  }
};

static String g_lastCmd;

class RXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string rx = pChar->getValue();
    if (rx.empty()) return;

    String cmd = String(rx.c_str());
    cmd.trim();
    cmd.toUpperCase();

    g_lastCmd = cmd;
    SERIAL_PORT.print("RX CMD: ");
    SERIAL_PORT.println(cmd);
  }
};

static void initBLE() {
  BLEDevice::init("ESP32_GLOVE_L");

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  txChar = service->createCharacteristic(
    CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic* rxChar = service->createCharacteristic(
    CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  rxChar->setCallbacks(new RXCallbacks());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  SERIAL_PORT.println("BLE ready, advertising as ESP32_GLOVE_L");
}

static void initIMU() {
  pinMode(IMU_PIN_CS, OUTPUT);
  digitalWrite(IMU_PIN_CS, HIGH);

  SPI_PORT.begin(IMU_PIN_SCK, IMU_PIN_MISO, IMU_PIN_MOSI, IMU_PIN_CS);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(IMU_PIN_CS, SPI_PORT);

    SERIAL_PORT.print("IMU init: ");
    SERIAL_PORT.println(myICM.statusString());

    if (myICM.status != ICM_20948_Stat_Ok) delay(500);
    else initialized = true;
  }

  SERIAL_PORT.println("IMU connected! Enabling DMP...");

  bool ok = true;
  ok &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  ok &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  ok &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  ok &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  ok &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  ok &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  ok &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (!ok) {
    bleSendLine("MSG,Enable DMP failed. Check ICM_20948_USE_DMP.");
    while (1) delay(1000);
  }

  bleSendLine("MSG,DMP enabled.");
}

/**
 * Calibration (all sensors at once):
 *  - Wait CAL_PREP_MS so the user can prepare
 *  - Hold all fingers bent for CAL_HOLD_MS -> record maximum values
 *  - Hold all fingers straight for CAL_HOLD_MS -> record minimum values
 */
static void calibrateAllAtOnce() {
  streamingEnabled = false;
  calibrated = false;

  bleSendLine("MSG,CALIBRATION START");
  bleSendLine("MSG,Prepare... calibration begins in 5 seconds.");
  delay(CAL_PREP_MS);

  for (int i = 0; i < 5; i++) {
    fingerMax[i] = 0;
    fingerMin[i] = 4095;
  }

  bleSendLine("MSG,PHASE 1/2: Hold ALL fingers BENT for 5 seconds...");
  unsigned long t1 = millis();
  while (millis() - t1 < CAL_HOLD_MS) {
    for (int i = 0; i < 5; i++) {
      int v = analogRead(fingerPins[i]);
      if (v > fingerMax[i]) fingerMax[i] = v;
    }
    delay(CAL_SAMPLE_MS);
  }

  bleSendLine("MSG,PHASE 2/2: Hold ALL fingers STRAIGHT for 5 seconds...");
  unsigned long t2 = millis();
  while (millis() - t2 < CAL_HOLD_MS) {
    for (int i = 0; i < 5; i++) {
      int v = analogRead(fingerPins[i]);
      if (v < fingerMin[i]) fingerMin[i] = v;
    }
    delay(CAL_SAMPLE_MS);
  }

  for (int i = 0; i < 5; i++) {
    if (fingerMax[i] < fingerMin[i]) {
      int tmp = fingerMin[i];
      fingerMin[i] = fingerMax[i];
      fingerMax[i] = tmp;
    }
  }

  char buf[200];

  snprintf(buf, sizeof(buf), "CAL,MIN,%d,%d,%d,%d,%d",
           fingerMin[0], fingerMin[1], fingerMin[2], fingerMin[3], fingerMin[4]);
  bleSendLine(buf);

  snprintf(buf, sizeof(buf), "CAL,MAX,%d,%d,%d,%d,%d",
           fingerMax[0], fingerMax[1], fingerMax[2], fingerMax[3], fingerMax[4]);
  bleSendLine(buf);

  bleSendLine("CALDONE");
  bleSendLine("MSG,Calibration complete. Streaming resumed.");
  calibrated = true;
  streamingEnabled = true;
}

static void processCmd(const String& cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "CAL") {
    calibrateAllAtOnce();
    return;
  }
  if (cmd == "STOP") {
    streamingEnabled = false;
    bleSendLine("MSG,Streaming stopped.");
    return;
  }
  if (cmd == "START") {
    streamingEnabled = true;
    bleSendLine("MSG,Streaming started.");
    return;
  }

  bleSendLine("MSG,Unknown command. Use CAL / START / STOP.");
}

void setup() {
  SERIAL_PORT.begin(SERIAL_BAUDRATE);
  delay(150);

  uint32_t t0 = millis();
  while (!SERIAL_PORT && (millis() - t0 < SERIAL_USB_WAIT_MS)) delay(10);

  SERIAL_PORT.println("Boot: Flex(5) + ICM-20948 (DMP SPI) + BLE NUS");

  analogReadResolution(ADC_RESOLUTION);
  for (int i = 0; i < 5; i++) pinMode(fingerPins[i], INPUT);

  initBLE();
  initIMU();

  bleSendLine("MSG,Connected. Send 'CAL' to calibrate (all fingers at once).");
  bleSendLine("MSG,Then it will stream: IMU,roll,pitch,yaw,S,r1..r5,N,n1..n5");
}

void loop() {
  if (g_lastCmd.length() > 0) {
    String cmd = g_lastCmd;
    g_lastCmd = "";
    processCmd(cmd);
  }

  if (!streamingEnabled) {
    delay(20);
    return;
  }

  int raw[5];
  float norm[5];

  for (int i = 0; i < 5; i++) {
    raw[i] = analogRead(fingerPins[i]);
    float n = map01(raw[i], fingerMin[i], fingerMax[i]);
    fingerNormF[i] = ema(fingerNormF[i], n, FLEX_FILTER_ALPHA);
    norm[i] = fingerNormF[i];
  }

  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {

      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;

      double s = 1.0 - ((q1*q1) + (q2*q2) + (q3*q3));
      if (s < 0.0) s = 0.0;
      double q0 = sqrt(s);

      double nrm = sqrt((q0*q0) + (q1*q1) + (q2*q2) + (q3*q3));
      if (nrm <= 0.0) {
        delay(1);
        return;
      }
      q0 /= nrm; q1 /= nrm; q2 /= nrm; q3 /= nrm;

      double qw = q0;
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      double t0r = +2.0 * (qw*qx + qy*qz);
      double t1r = +1.0 - 2.0 * (qx*qx + qy*qy);
      double roll  = atan2(t0r, t1r) * 180.0 / PI;

      double t2p = +2.0 * (qw*qy - qx*qz);
      if (t2p > 1.0) t2p = 1.0;
      if (t2p < -1.0) t2p = -1.0;
      double pitch = asin(t2p) * 180.0 / PI;

      double t3y = +2.0 * (qw*qz + qx*qy);
      double t4y = +1.0 - 2.0 * (qy*qy + qz*qz);
      double yaw = atan2(t3y, t4y) * 180.0 / PI;

      if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw)) {
        delay(1);
        return;
      }

      float roll_u  = (float)roll;
      float pitch_u = (float)pitch;
      float yaw_u   = unwrapYaw((float)yaw);

      roll_f  = ema(roll_f,  roll_u,  ANGLE_FILTER_ALPHA);
      pitch_f = ema(pitch_f, pitch_u, ANGLE_FILTER_ALPHA);
      yaw_f   = ema(yaw_f,   yaw_u,   ANGLE_FILTER_ALPHA);

      char line[260];
      snprintf(
        line, sizeof(line),
        "IMU,%.2f,%.2f,%.2f,S,%d,%d,%d,%d,%d,N,%.3f,%.3f,%.3f,%.3f,%.3f",
        roll_f, pitch_f, yaw_f,
        raw[0], raw[1], raw[2], raw[3], raw[4],
        norm[0], norm[1], norm[2], norm[3], norm[4]
      );
      bleSendLine(line);
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
    delay(LOOP_DELAY_MS);
  }
}