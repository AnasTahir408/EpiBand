// ============================================================
//  FALL + SEIZURE DETECTION SYSTEM  v4.0  —  ESP32-WROOM-32D
//  Hardware : ESP32-WROOM-32D + MPU6050
//  Comms    : BLE (Bluetooth Low Energy) — NOTIFY on seizure
//  Planned  : MAX30102 (heart rate display only, not detection)
//
//  BLE BEHAVIOUR:
//    Sends "1" (notify) → seizure confirmed, buzzer active  → triggers call on app
//    Sends "0" (notify) → event cancelled / system reset    → app clears alert
//
//  NOTE: BluetoothSerial (Classic BT) is NOT used here.
//        BLE and Classic BT cannot run simultaneously on ESP32.
//        ESP32-WROOM-32D fully supports BLE — no hardware issue.
//
//  WIRING:
//    MPU6050 SDA  → GPIO 21
//    MPU6050 SCL  → GPIO 22
//    MPU6050 VCC  → 3.3V
//    MPU6050 GND  → GND
//    Button       → GPIO 34  (input-only pin — needs external 10kΩ pull-up to 3.3V)
//    Buzzer       → GPIO 25  (LEDC PWM channel 0)
//    LED1 (fall)  → GPIO 26
//    LED2 (shake) → GPIO 27
//    LED3 (warn)  → GPIO 32
//
//  AVOIDED PINS (ESP32 strapping pins — do not use for peripherals):
//    GPIO 0, 2, 5, 12, 15
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>   // Required for NOTIFY — adds the Client Characteristic Config descriptor

// ===================== BLE CONFIGURATION ===================
// Use full 128-bit UUIDs for reliable cross-platform compatibility.
// You can keep these values — just match them in your app.
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

BLECharacteristic *pCharacteristic = nullptr;
bool               bleClientConnected    = false;
bool               blePrevConnected      = false;
BLEServer         *pServer         = nullptr;

// ============== BLE SERVER CALLBACKS =======================
// Tracks whether a phone/app is currently connected
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pSvr) override {
    bleClientConnected = true;
    Serial.println("[BLE] Client connected");
  }
  void onDisconnect(BLEServer *pSvr) override {
    bleClientConnected = false;
    Serial.println("[BLE] Client disconnected — restarting advertising");
  }
};

// ===================== PIN DEFINITIONS =====================
const int PIN_SDA    = 21;
const int PIN_SCL    = 22;
const int PIN_BUTTON = 34;   // Input-only GPIO — external 10kΩ pull-up to 3.3V required
const int PIN_BUZZER = 25;
const int PIN_LED1   = 26;
const int PIN_LED2   = 27;
const int PIN_LED3   = 32;

// LEDC buzzer  (ESP32 Arduino core v3.x API — no channel number needed)
const int BUZZER_LEDC_FREQ = 2000;  // Hz  — audible tone
const int BUZZER_LEDC_RES  = 8;     // bits (0–255 duty)
const int BUZZER_DUTY_ON   = 128;   // 50% duty
const int BUZZER_DUTY_OFF  = 0;

// =================== FALL DETECTION ========================
const float FREE_FALL_THRESHOLD   = 0.5;
const float IMPACT_THRESHOLD      = 3.0;
const unsigned long FREE_FALL_MIN_TIME = 100;

enum FallState { NORMAL, FREE_FALL_DETECTED, IMPACT_DETECTED, FALL_CONFIRMED };
FallState currentState = NORMAL;

unsigned long freeFallStartTime = 0;

// ============= SEIZURE / TREMOR DETECTION ==================
//
// DESIGN PHILOSOPHY:
//   Seizures (tonic-clonic) produce repetitive, violent muscle jerks
//   driven by rapid nerve discharges — typically 3–5 Hz bursts.
//   We model this with a sliding-window shake counter:
//     • Each 1-second window votes YES/NO based on shake count.
//     • Consecutive YES windows increment a counter.
//     • Seizure confirmed when YES windows fill a 20-second sustained period.
//   This mirrors the clinical criterion of sustained rhythmic activity
//   lasting ≥20 seconds used to flag a generalised convulsive event.
//
const float    SEIZURE_SHAKE_THRESHOLD = 1.5;
const float    SEIZURE_GYRO_THRESHOLD  = 100.0;
const unsigned long SEIZURE_WINDOW     = 1000;
const int      SEIZURE_MIN_SHAKES      = 2;
const int      SEIZURE_WINDOWS_NEEDED  = 3;    // 3 consecutive 1s windows = confirmed (TEST VALUE — raise to 20 for deployment)

unsigned long windowStartTime    = 0;
int           shakeCount         = 0;
int           consecutiveWindows = 0;
bool          lastShake          = false;
float         lastAccelMag       = 1.0;
unsigned long lastShakeTime      = 0;
bool          seizureDetected    = false;
unsigned long seizureStartTime   = 0;

// =================== SYSTEM MODE ===========================
enum SystemMode { FALL_MODE, SEIZURE_MODE };
SystemMode mode = FALL_MODE;

// ============== BUZZER / LED / BUTTON STATE ================
bool          buzzerActive    = false;
unsigned long buzzerStartTime = 0;
const unsigned long BUZZER_DURATION = 20000; // 20s cancellation window — BLE fires only if not cancelled in time

bool led1On = false;
bool led2On = false;
bool led3On = false;

// ================== AUTO-RESET TIMER =======================
const unsigned long FALL_NO_SEIZURE_TIMEOUT = 20000;
unsigned long fallConfirmedTime = 0;

// =================== SERIAL MONITOR ========================
unsigned long lastPrint = 0;

// ============================================================
Adafruit_MPU6050 mpu;

// ============================================================
//  BLE HELPER — send flag to connected app
//  flag "1" = seizure alert (trigger call on app)
//  flag "0" = alert cleared (cancel / reset)
// ============================================================

void bleSendFlag(const char *flag) {
  if (pCharacteristic == nullptr) return;
  pCharacteristic->setValue(flag);
  pCharacteristic->notify();
  Serial.print("[BLE] Sent flag: ");
  Serial.println(flag);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Don't block forever if no Serial monitor
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  // I2C — explicit pins required on ESP32
  Wire.begin(PIN_SDA, PIN_SCL);

  // GPIO
  // GPIO 34 is input-only silicon — INPUT_PULLUP has no effect.
  // You MUST wire a 10kΩ resistor from GPIO 34 to 3.3V on the breadboard.
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_LED1,   OUTPUT);
  pinMode(PIN_LED2,   OUTPUT);
  pinMode(PIN_LED3,   OUTPUT);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  digitalWrite(PIN_LED3, LOW);

  // LEDC buzzer — core v3.x: ledcAttach(pin, freq, resolution) replaces
  // ledcSetup() + ledcAttachPin(). ledcWrite() now takes pin, not channel.
  ledcAttach(PIN_BUZZER, BUZZER_LEDC_FREQ, BUZZER_LEDC_RES);
  ledcWrite(PIN_BUZZER, BUZZER_DUTY_OFF);

  // ---- BLE INIT ----
  BLEDevice::init("EpiGuard");                         // Device name shown on phone
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  // BLE2902 descriptor is mandatory for NOTIFY to work on iOS and Android
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);  // app can filter by service UUID
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);         // helps with iPhone connection stability
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising started — device: EpiGuard");

  // ---- MPU6050 ----
  Serial.println("\n================ SYSTEM BOOT ================");
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 NOT FOUND. Check SDA/SCL wiring.");
    while (1) {
      digitalWrite(PIN_LED1, HIGH); delay(150);
      digitalWrite(PIN_LED1, LOW);  delay(150);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 OK");
  Serial.println("SYSTEM READY — FALL MODE");
  Serial.println("=============================================\n");
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {

  // ---- BLE reconnect: restart advertising after disconnect ----
  if (!bleClientConnected && blePrevConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("[BLE] Advertising restarted");
    blePrevConnected = false;
  }
  if (bleClientConnected && !blePrevConnected) {
    blePrevConnected = true;
  }

  // ---- Sensor read ----
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x / 9.81;
  float ay = a.acceleration.y / 9.81;
  float az = a.acceleration.z / 9.81;
  float accelMag = sqrt(ax*ax + ay*ay + az*az);

  float gx = g.gyro.x * 180.0 / PI;
  float gy = g.gyro.y * 180.0 / PI;
  float gz = g.gyro.z * 180.0 / PI;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // ---- Button (highest priority) ----
  checkButton();

  // ---- Detection logic ----
  if (mode == FALL_MODE) {
    runFall(accelMag);
  } else {
    runSeizure(accelMag, gyroMag);
    checkFallNoSeizureTimeout();
  }

  // ---- Buzzer ----
  manageBuzzer();

  // ---- LEDs ----
  updateLEDs();

  // ---- Serial monitor ----
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    printStatus(accelMag, gyroMag);
  }

  delay(10);
}

// ============================================================
//  BUTTON HANDLER
//  GPIO 34 — input-only, external pull-up required, LOW = pressed
// ============================================================

void checkButton() {
  if (digitalRead(PIN_BUTTON) == LOW) {
    delay(50);
    if (digitalRead(PIN_BUTTON) == LOW) {
      Serial.println("\n[BUTTON] Event cancelled — returning to FALL MODE\n");
      bleSendFlag("0");   // tell app: alert cleared
      fullSystemReset();
      while (digitalRead(PIN_BUTTON) == LOW);
    }
  }
}

// ============================================================
//  FULL SYSTEM RESET
// ============================================================

void fullSystemReset() {
  mode         = FALL_MODE;
  currentState = NORMAL;

  ledcWrite(PIN_BUZZER, BUZZER_DUTY_OFF);
  buzzerActive = false;

  led1On = led2On = led3On = false;
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  digitalWrite(PIN_LED3, LOW);

  freeFallStartTime  = 0;
  fallConfirmedTime  = 0;
  windowStartTime    = millis();
  shakeCount         = 0;
  consecutiveWindows = 0;
  seizureDetected    = false;
  lastShake          = false;
  lastAccelMag       = 1.0;
  lastShakeTime      = 0;
}

// ============================================================
//  FALL DETECTION STATE MACHINE
//
//  Simplified to two stages: FREE_FALL → IMPACT.
//  On confirmed impact we hand off to SEIZURE_MODE immediately —
//  no inactivity wait, no tilt check. Seizures can begin at or
//  before impact, so gating entry on stillness would suppress the
//  very shaking we need to catch. The 20s auto-reset in
//  checkFallNoSeizureTimeout() handles the false-positive case.
// ============================================================

void runFall(float accelMag) {
  switch (currentState) {

    case NORMAL:
      if (accelMag < FREE_FALL_THRESHOLD) {
        freeFallStartTime = millis();
        currentState = FREE_FALL_DETECTED;
        Serial.println("[EVENT] Free-fall phase detected");
      }
      break;

    case FREE_FALL_DETECTED:
      if (accelMag > IMPACT_THRESHOLD) {
        if (millis() - freeFallStartTime >= FREE_FALL_MIN_TIME) {
          // Impact confirmed — hand off to seizure monitor immediately
          currentState      = FALL_CONFIRMED;
          fallConfirmedTime = millis();
          led1On            = true;

          Serial.println("[EVENT] Impact detected");
          Serial.println("\n!!!!!!!! FALL CONFIRMED !!!!!!!!");
          Serial.println(">>> Switching to SEIZURE MONITOR MODE\n");

          mode = SEIZURE_MODE;

          windowStartTime    = millis();
          shakeCount         = 0;
          consecutiveWindows = 0;
          seizureDetected    = false;
          lastShake          = false;
          lastAccelMag       = accelMag;

        } else {
          currentState = NORMAL; // spike too brief — noise
        }
      }
      // Guard: implausibly long freefall with no impact → reset
      if (millis() - freeFallStartTime > 2000) {
        currentState = NORMAL;
      }
      break;

    case IMPACT_DETECTED:
      // State kept in enum for continuity but no longer entered.
      // Fall through to safety reset.
      currentState = NORMAL;
      break;

    case FALL_CONFIRMED:
      break; // safety guard
  }
}

// ============================================================
//  AUTO-RESET: fall with no seizure in 20 s
// ============================================================

void checkFallNoSeizureTimeout() {
  if (!seizureDetected && !buzzerActive) {
    if (millis() - fallConfirmedTime > FALL_NO_SEIZURE_TIMEOUT) {
      Serial.println("[INFO] No seizure after fall — auto-resetting");
      fullSystemReset();
    }
  }
}

// ============================================================
//  SEIZURE / TREMOR DETECTION
// ============================================================

void runSeizure(float accelMag, float gyroMag) {

  float accelChange = abs(accelMag - lastAccelMag);
  lastAccelMag = accelMag;

  bool shakeNow = (accelChange > SEIZURE_SHAKE_THRESHOLD) ||
                  (gyroMag     > SEIZURE_GYRO_THRESHOLD);

  if (shakeNow) {
    if (!lastShake) {
      shakeCount++;
      lastShakeTime = millis();
      lastShake     = true;
    }
  } else {
    lastShake = false;
  }

  // ---- Window evaluation ----
  if (millis() - windowStartTime >= SEIZURE_WINDOW) {

    if (shakeCount >= SEIZURE_MIN_SHAKES) {
      consecutiveWindows++;

      if (consecutiveWindows == SEIZURE_WINDOWS_NEEDED / 3) {
        led2On = true;
        Serial.println("[STAGE] Shakes accumulating — LED2 ON");
      }

      if (consecutiveWindows == (SEIZURE_WINDOWS_NEEDED * 2) / 3) {
        led3On = true;
        Serial.println("[STAGE] Warning — buzzer imminent — LED3 ON");
      }

      // ---- ALL 3 STAGES COMPLETE: fire buzzer + send BLE flag "1" ----
      if (consecutiveWindows >= SEIZURE_WINDOWS_NEEDED && !seizureDetected) {
        seizureDetected  = true;
        seizureStartTime = millis();

        Serial.println("\n!!!!!!!! SEIZURE CONFIRMED !!!!!!!!\n");

        // Start buzzer — this is the 20s cancellation window.
        // BLE flag "1" is deliberately NOT sent here.
        // It is only sent in manageBuzzer() if the full 20s elapses
        // without the button being pressed. Button press calls
        // fullSystemReset() which sends "0" and aborts delivery.
        buzzerActive    = true;
        buzzerStartTime = millis();
        ledcWrite(PIN_BUZZER, BUZZER_DUTY_ON);
        Serial.println("[BUZZER] 20s cancellation window started — press button to cancel");
      }

    } else {
      if (consecutiveWindows > 0) {
        Serial.print("[INFO] Tremor streak broken at ");
        Serial.print(consecutiveWindows);
        Serial.println(" windows — resetting");
      }
      consecutiveWindows = 0;
      led2On = false;
      led3On = false;
    }

    windowStartTime = millis();
    shakeCount      = 0;
  }

  // ---- Seizure stopped: no activity for 5 s ----
  if (seizureDetected && (millis() - lastShakeTime > 5000)) {
    Serial.println("[EVENT] Seizure activity stopped");
    seizureDetected    = false;
    consecutiveWindows = 0;
    led2On = false;
    led3On = false;
    // Buzzer still runs its full 20s cancellation window independently
  }
}

// ============================================================
//  BUZZER MANAGEMENT
// ============================================================

void manageBuzzer() {
  if (buzzerActive) {
    if (millis() - buzzerStartTime >= BUZZER_DURATION) {
      ledcWrite(PIN_BUZZER, BUZZER_DUTY_OFF);
      buzzerActive = false;
      Serial.println("[BUZZER] 20s elapsed — no cancellation received");

      // Button was NOT pressed in time → deliver BLE alert now
      // App receives "1" and triggers the emergency call
      Serial.println("[BLE] Sending alert flag — emergency call triggered");
      bleSendFlag("1");
    }
  }
}

// ============================================================
//  LED STAGES
// ============================================================

void updateLEDs() {
  digitalWrite(PIN_LED1, led1On ? HIGH : LOW);
  digitalWrite(PIN_LED2, led2On ? HIGH : LOW);
  digitalWrite(PIN_LED3, led3On ? HIGH : LOW);
}

// ============================================================
//  SERIAL STATUS
// ============================================================

void printStatus(float accelMag, float gyroMag) {
  Serial.println("------------------------------------------------");
  Serial.print("MODE : ");
  Serial.println(mode == FALL_MODE ? "FALL MODE" : "SEIZURE MODE");
  Serial.print("STATE: ");
  Serial.println(getFallStateString());
  Serial.print("Accel: "); Serial.print(accelMag, 2);
  Serial.print(" g  |  Gyro: "); Serial.print(gyroMag, 1);
  Serial.println(" dps");
  Serial.print("BLE client: ");
  Serial.println(bleClientConnected ? "CONNECTED" : "waiting...");

  if (mode == SEIZURE_MODE) {
    Serial.print("Shakes(window): "); Serial.print(shakeCount);
    Serial.print("  |  Windows: "); Serial.print(consecutiveWindows);
    Serial.print("/"); Serial.print(SEIZURE_WINDOWS_NEEDED);
    Serial.print("  |  Seizure: ");
    Serial.println(seizureDetected ? "CONFIRMED" : "monitoring...");
    Serial.print("Buzzer: ");
    Serial.println(buzzerActive ? "ON" : "OFF");

    if (fallConfirmedTime > 0 && !seizureDetected && !buzzerActive) {
      unsigned long elapsed   = millis() - fallConfirmedTime;
      unsigned long remaining = (elapsed < FALL_NO_SEIZURE_TIMEOUT)
                                ? (FALL_NO_SEIZURE_TIMEOUT - elapsed) / 1000
                                : 0;
      Serial.print("Auto-reset in: ");
      Serial.print(remaining);
      Serial.println("s");
    }
  }
  Serial.println("------------------------------------------------\n");
}

// ============================================================
//  HELPER
// ============================================================

String getFallStateString() {
  switch (currentState) {
    case NORMAL:             return "NORMAL";
    case FREE_FALL_DETECTED: return "FREE_FALL";
    case IMPACT_DETECTED:    return "IMPACT";
    case FALL_CONFIRMED:     return "FALL_CONFIRMED";
    default:                 return "UNKNOWN";
  }
}

// ============================================================
//  FUTURE STUBS
// ============================================================

// void readHeartRate() {
//   // MAX30102 — read BPM and send over BLE as separate characteristic
//   // bleSendFlag() is for alert flags only; HR needs its own characteristic
// }
