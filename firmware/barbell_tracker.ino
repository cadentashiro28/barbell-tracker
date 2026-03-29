#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ─── Configuration ────────────────────────────────────────────────────────────

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcd1234-ab12-ab12-ab12-abcdef123456"

#define SAMPLE_RATE_MS  10    // 100 Hz
#define BATCH_SIZE      5
#define CALIBRATE       false

// ─── Globals ──────────────────────────────────────────────────────────────────

Adafruit_MPU6050 mpu;
BLECharacteristic *pCharacteristic;

bool deviceConnected = false;
bool wasConnected    = false;

float az_offset = 0.228662;

static float batch_ax[BATCH_SIZE];
static float batch_ay[BATCH_SIZE];
static float batch_az[BATCH_SIZE];
static float batch_gx[BATCH_SIZE];
static float batch_gy[BATCH_SIZE];
static float batch_gz[BATCH_SIZE];
static int   batchIdx = 0;

// ─── BLE Callbacks ────────────────────────────────────────────────────────────

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) {
    deviceConnected = true;
    wasConnected    = true;
    Serial.println("Client connected");
  }
  void onDisconnect(BLEServer* s) {
    deviceConnected = false;
    Serial.println("Client disconnected");
  }
};

// ─── Calibration ──────────────────────────────────────────────────────────────

void calibrate() {
  Serial.println("Calibrating... keep sensor still");
  delay(2000);

  const int samples = 500;
  float     sum     = 0.0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += a.acceleration.z;
    delay(2);
  }

  az_offset = (sum / samples) - 9.81;
  Serial.print("Calibrated az_offset: ");
  Serial.println(az_offset, 6);
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.println("MPU6050 initialized");

  if (CALIBRATE) calibrate();

  BLEDevice::init("ESP32-Squat");
  BLEDevice::setMTU(185);

  BLEServer  *server  = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  pCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();
  Serial.println("BLE advertising...");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void sendBatch() {
  char buf[512];
  snprintf(buf, sizeof(buf),
    "%lu,"
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    millis(),
    batch_ax[0], batch_ay[0], batch_az[0], batch_gx[0], batch_gy[0], batch_gz[0],
    batch_ax[1], batch_ay[1], batch_az[1], batch_gx[1], batch_gy[1], batch_gz[1],
    batch_ax[2], batch_ay[2], batch_az[2], batch_gx[2], batch_gy[2], batch_gz[2],
    batch_ax[3], batch_ay[3], batch_az[3], batch_gx[3], batch_gy[3], batch_gz[3],
    batch_ax[4], batch_ay[4], batch_az[4], batch_gx[4], batch_gy[4], batch_gz[4]
  );
  pCharacteristic->setValue((uint8_t*)buf, strlen(buf));
  pCharacteristic->notify();
}

void loop() {
  // Handle reconnect on main thread
  if (!deviceConnected && wasConnected) {
    delay(1000);
    BLEDevice::getAdvertising()->start();
    Serial.println("Advertising restarted");
    wasConnected = false;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Debug single axis to serial
  Serial.println(a.acceleration.z - az_offset);

  if (deviceConnected) {
    batch_ax[batchIdx] = a.acceleration.x;
    batch_ay[batchIdx] = a.acceleration.y;
    batch_az[batchIdx] = a.acceleration.z - az_offset;
    batch_gx[batchIdx] = g.gyro.x;  // rad/s
    batch_gy[batchIdx] = g.gyro.y;
    batch_gz[batchIdx] = g.gyro.z;
    batchIdx++;

    if (batchIdx >= BATCH_SIZE) {
      sendBatch();
      batchIdx = 0;
    }
  }

  delay(SAMPLE_RATE_MS);
}