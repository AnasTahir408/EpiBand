// ============================================================
//  FALL + SEIZURE DETECTION SYSTEM  v5.0  —  ESP32-WROOM-32D
//  Hardware : ESP32-WROOM-32D + MPU6050 + SSD1306 OLED 0.96"
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
//    MPU6050 SDA   → GPIO 21
//    MPU6050 SCL   → GPIO 22
//    OLED SDA      → GPIO 21  (shared I2C bus with MPU6050)
//    OLED SCL      → GPIO 22  (shared I2C bus with MPU6050)
//    OLED VCC      → 3.3V
//    OLED GND      → GND
//    MPU6050 VCC   → 3.3V
//    MPU6050 GND   → GND
//    Button        → GPIO 18  (use INPUT_PULLUP — LOW = pressed)
//    Buzzer        → GPIO 25  (LEDC PWM channel 0)
//
//  OLED I2C ADDRESS: 0x3C (default for most 0.96" SSD1306 modules)
//
//  AVOIDED PINS (ESP32 strapping pins — do not use for peripherals):
//    GPIO 0, 2, 5, 12, 15
//
//  REQUIRED LIBRARIES (install via Arduino Library Manager):
//    - Adafruit SSD1306
//    - Adafruit GFX Library
//    - Adafruit MPU6050
//    - Adafruit BusIO
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ===================== BLE CONFIGURATION ===================
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

BLECharacteristic *pCharacteristic = nullptr;
bool               bleClientConnected = false;
bool               blePrevConnected   = false;
BLEServer         *pServer            = nullptr;

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
const int PIN_BUTTON = 18;   // Standard GPIO — INPUT_PULLUP supported, LOW = pressed
const int PIN_BUZZER = 25;

// LEDC buzzer
const int BUZZER_LEDC_FREQ = 2000;
const int BUZZER_LEDC_RES  = 8;
const int BUZZER_DUTY_ON   = 128;
const int BUZZER_DUTY_OFF  = 0;

// ===================== OLED CONFIGURATION ==================
#define OLED_WIDTH    128
#define OLED_HEIGHT    64
#define OLED_ADDRESS  0x3C   // Default I2C address for most 0.96" SSD1306 modules
                              // If display blank, try 0x3D

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// =================== FALL DETECTION ========================
const float FREE_FALL_THRESHOLD       = 0.5;
const float IMPACT_THRESHOLD          = 3.0;
const unsigned long FREE_FALL_MIN_TIME = 100;

enum FallState { NORMAL, FREE_FALL_DETECTED, IMPACT_DETECTED, FALL_CONFIRMED };
FallState currentState = NORMAL;

unsigned long freeFallStartTime = 0;

// ============= SEIZURE / TREMOR DETECTION ==================
const float    SEIZURE_SHAKE_THRESHOLD = 1.5;
const float    SEIZURE_GYRO_THRESHOLD  = 100.0;
const unsigned long SEIZURE_WINDOW     = 1000;
const int      SEIZURE_MIN_SHAKES      = 2;
const int      SEIZURE_WINDOWS_NEEDED  = 3;   // TEST VALUE — raise to 20 for deployment

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

// ============== BUZZER / BUTTON STATE ======================
bool          buzzerActive    = false;
unsigned long buzzerStartTime = 0;
const unsigned long BUZZER_DURATION = 20000;

// ================== AUTO-RESET TIMER =======================
const unsigned long FALL_NO_SEIZURE_TIMEOUT = 20000;
unsigned long fallConfirmedTime = 0;

// =================== OLED STATE TRACKING ===================
// Tracks the last string shown so we only redraw on change
String lastOledLine1 = "";
String lastOledLine2 = "";
String lastOledLine3 = "";

// =================== SERIAL MONITOR ========================
unsigned long lastPrint = 0;

// ============================================================
Adafruit_MPU6050 mpu;

// ============================================================
//  BLE HELPER
// ============================================================

void bleSendFlag(const char *flag) {
  if (pCharacteristic == nullptr) return;
  pCharacteristic->setValue(flag);
  pCharacteristic->notify();
  Serial.print("[BLE] Sent flag: ");
  Serial.println(flag);
}

// ============================================================
//  OLED HELPERS
// ============================================================

// Draw the EpiGuard logo/header on the top bar
void oledDrawHeader() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(28, 0);
  display.print("[ EpiGuard ]");
  display.drawLine(0, 10, OLED_WIDTH - 1, 10, SSD1306_WHITE);
}

// Main OLED update — call whenever state changes.
// line1 : big status label (e.g. "NORMAL", "FREE FALL")
// line2 : sub-info line   (e.g. "Monitoring..." or "BLE: CONNECTED")
// line3 : bottom detail   (e.g. seizure window progress)
// invertDisplay: true to flash inverted colours for critical alerts
void oledUpdate(String line1, String line2, String line3, bool invertDisplay = false) {

  // Skip redraw if nothing changed (saves ~5 ms per loop)
  if (line1 == lastOledLine1 && line2 == lastOledLine2 && line3 == lastOledLine3) return;

  lastOledLine1 = line1;
  lastOledLine2 = line2;
  lastOledLine3 = line3;

  display.clearDisplay();
  display.invertDisplay(invertDisplay);

  oledDrawHeader();

  // Large status label — centred
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(line1, 0, 0, &x1, &y1, &w, &h);
  int cx = max(0, (OLED_WIDTH - (int)w) / 2);
  display.setCursor(cx, 14);
  display.print(line1);

  // Sub-info line
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.print(line2);

  // Bottom detail line
  display.setCursor(0, 52);
  display.print(line3);

  display.display();
}

// Splash screen shown once at boot
void oledSplash() {
  display.clearDisplay();
  display.invertDisplay(false);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 8);
  display.print("EpiGuard");
  display.setTextSize(1);
  display.setCursor(22, 30);
  display.print("Fall & Seizure");
  display.setCursor(28, 42);
  display.print("Detection");
  display.setCursor(18, 54);
  display.print("v5.0  Booting...");
  display.display();
  delay(2000);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  // I2C — both MPU6050 and OLED share this bus
  Wire.begin(PIN_SDA, PIN_SCL);

  // GPIO
  pinMode(PIN_BUTTON, INPUT_PULLUP);   // GPIO 18 supports pull-up; LOW = pressed

  // LEDC buzzer
  ledcAttach(PIN_BUZZER, BUZZER_LEDC_FREQ, BUZZER_LEDC_RES);
  ledcWrite(PIN_BUZZER, BUZZER_DUTY_OFF);

  // ---- OLED INIT ----
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("ERROR: OLED SSD1306 NOT FOUND. Check wiring / I2C address.");
    // Blink buzzer short pulses as error indicator
    while (1) {
      ledcWrite(PIN_BUZZER, BUZZER_DUTY_ON); delay(100);
      ledcWrite(PIN_BUZZER, BUZZER_DUTY_OFF); delay(200);
    }
  }
  display.clearDisplay();
  display.display();
  oledSplash();

  // ---- BLE INIT ----
  BLEDevice::init("EpiGuard");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising started — device: EpiGuard");

  // ---- MPU6050 INIT ----
  Serial.println("\n================ SYSTEM BOOT ================");
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 NOT FOUND. Check SDA/SCL wiring.");
    oledUpdate("MPU ERR", "Check SDA/SCL", "System halted");
    while (1) {
      ledcWrite(PIN_BUZZER, BUZZER_DUTY_ON); delay(150);
      ledcWrite(PIN_BUZZER, BUZZER_DUTY_OFF); delay(150);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 OK");
  Serial.println("SYSTEM READY — FALL MODE");
  Serial.println("=============================================\n");

  windowStartTime = millis();

  // Initial OLED state
  oledUpdate("NORMAL", "Fall mode active", bleClientConnected ? "BLE: ON" : "BLE: waiting");
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {

  // ---- BLE reconnect ----
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

  // ---- Button ----
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

  // ---- OLED refresh ----
  updateOled();

  // ---- Serial monitor ----
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    printStatus(accelMag, gyroMag);
  }

  delay(10);
}

// ============================================================
//  BUTTON HANDLER — GPIO 18, INPUT_PULLUP, LOW = pressed
// ============================================================

void checkButton() {
  if (digitalRead(PIN_BUTTON) == LOW) {
    delay(50);
    if (digitalRead(PIN_BUTTON) == LOW) {
      Serial.println("\n[BUTTON] Event cancelled — returning to FALL MODE\n");
      bleSendFlag("0");
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

  freeFallStartTime  = 0;
  fallConfirmedTime  = 0;
  windowStartTime    = millis();
  shakeCount         = 0;
  consecutiveWindows = 0;
  seizureDetected    = false;
  lastShake          = false;
  lastAccelMag       = 1.0;
  lastShakeTime      = 0;

  // Force OLED redraw after reset
  lastOledLine1 = "";
  lastOledLine2 = "";
  lastOledLine3 = "";
  display.invertDisplay(false);
  oledUpdate("NORMAL", "Fall mode active", bleClientConnected ? "BLE: ON" : "BLE: waiting");
}

// ============================================================
//  FALL DETECTION STATE MACHINE
// ============================================================

void runFall(float accelMag) {
  switch (currentState) {

    case NORMAL:
      if (accelMag < FREE_FALL_THRESHOLD) {
        freeFallStartTime = millis();
        currentState = FREE_FALL_DETECTED;
        Serial.println("[EVENT] Free-fall phase detected");
        // OLED updated via updateOled() — no duplicate call needed
      }
      break;

    case FREE_FALL_DETECTED:
      if (accelMag > IMPACT_THRESHOLD) {
        if (millis() - freeFallStartTime >= FREE_FALL_MIN_TIME) {
          currentState      = FALL_CONFIRMED;
          fallConfirmedTime = millis();

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
          currentState = NORMAL;
        }
      }
      if (millis() - freeFallStartTime > 2000) {
        currentState = NORMAL;
      }
      break;

    case IMPACT_DETECTED:
      currentState = NORMAL;
      break;

    case FALL_CONFIRMED:
      break;
  }
}

// ============================================================
//  AUTO-RESET: fall with no seizure in 20s
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

  if (millis() - windowStartTime >= SEIZURE_WINDOW) {

    if (shakeCount >= SEIZURE_MIN_SHAKES) {
      consecutiveWindows++;

      if (consecutiveWindows == SEIZURE_WINDOWS_NEEDED / 3) {
        Serial.println("[STAGE] Shakes accumulating");
      }
      if (consecutiveWindows == (SEIZURE_WINDOWS_NEEDED * 2) / 3) {
        Serial.println("[STAGE] Warning — buzzer imminent");
      }

      if (consecutiveWindows >= SEIZURE_WINDOWS_NEEDED && !seizureDetected) {
        seizureDetected  = true;
        seizureStartTime = millis();

        Serial.println("\n!!!!!!!! SEIZURE CONFIRMED !!!!!!!!\n");

        buzzerActive    = true;
        buzzerStartTime = millis();
        ledcWrite(PIN_BUZZER, BUZZER_DUTY_ON);
        Serial.println("[BUZZER] 20s cancellation window started — press button to cancel");

        // Force OLED redraw on seizure confirmation
        lastOledLine1 = "";
      }

    } else {
      if (consecutiveWindows > 0) {
        Serial.print("[INFO] Tremor streak broken at ");
        Serial.print(consecutiveWindows);
        Serial.println(" windows — resetting");
        lastOledLine1 = ""; // Force redraw
      }
      consecutiveWindows = 0;
    }

    windowStartTime = millis();
    shakeCount      = 0;
  }

  if (seizureDetected && (millis() - lastShakeTime > 5000)) {
    Serial.println("[EVENT] Seizure activity stopped");
    seizureDetected    = false;
    consecutiveWindows = 0;
    lastOledLine1 = ""; // Force redraw
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
      Serial.println("[BLE] Sending alert flag — emergency call triggered");
      bleSendFlag("1");
      lastOledLine1 = ""; // Force OLED to show ALERT state
    }
  }
}

// ============================================================
//  OLED STATE DISPLAY
//  Maps current system state to a human-readable OLED screen.
//  Called every loop() cycle; internal diffing avoids flicker.
// ============================================================

void updateOled() {
  String line1, line2, line3;
  bool   invert = false;

  String bleStatus = bleClientConnected ? "BLE: Connected" : "BLE: Waiting...";

  // ── ALERT GENERATED (buzzer timed out, BLE "1" sent) ──────
  // Detected by: buzzer was active but now off, and we are still in SEIZURE_MODE
  // We track this by checking mode + !buzzerActive + seizure history
  // A simpler flag: after bleSendFlag("1") we use a dedicated bool
  // (see alertSent flag below — added for clarity)
  static bool alertSent = false;

  if (buzzerActive) {
    // ── SEIZURE CONFIRMED + cancellation window open ──────────
    unsigned long elapsed   = millis() - buzzerStartTime;
    unsigned long remaining = (BUZZER_DURATION - elapsed) / 1000;
    line1  = "SEIZURE";
    line2  = "Alert in " + String(remaining) + "s  Btn=Cancel";
    line3  = bleStatus;
    invert = true;
    alertSent = false;

  } else if (alertSent) {
    // ── ALERT HAS BEEN TRANSMITTED ────────────────────────────
    line1  = "ALERT!";
    line2  = "Emergency call sent";
    line3  = bleStatus;
    invert = true;

  } else if (mode == SEIZURE_MODE) {

    if (seizureDetected) {
      // Seizure activity ongoing but buzzer not yet started
      // (edge case during window evaluation)
      line1 = "SEIZURE";
      line2 = "Activity detected";
      line3 = "Win: " + String(consecutiveWindows) + "/" + String(SEIZURE_WINDOWS_NEEDED)
              + "  " + bleStatus;
      invert = true;

    } else {
      // Fall confirmed, monitoring for seizure
      String winProgress = "Win: " + String(consecutiveWindows) + "/" + String(SEIZURE_WINDOWS_NEEDED);

      if (consecutiveWindows == 0) {
        line1 = "IMPACT";
        line2 = "Seizure monitor ON";
        line3 = winProgress + "  " + bleStatus;
        invert = false;

      } else if (consecutiveWindows < SEIZURE_WINDOWS_NEEDED) {
        // Shakes building up
        line1 = "SHAKING";
        line2 = "Tremor detected";
        line3 = winProgress + "  " + bleStatus;
        invert = false;
      }
    }

  } else {
    // FALL_MODE
    switch (currentState) {
      case NORMAL:
        line1  = "NORMAL";
        line2  = "Monitoring fall...";
        line3  = bleStatus;
        invert = false;
        break;

      case FREE_FALL_DETECTED:
        line1  = "FREE FALL";
        line2  = "Fall in progress";
        line3  = bleStatus;
        invert = false;
        break;

      case IMPACT_DETECTED:
        // Transitional — rarely renders, but handle it
        line1  = "IMPACT";
        line2  = "Impact registered";
        line3  = bleStatus;
        invert = false;
        break;

      case FALL_CONFIRMED:
        line1  = "IMPACT";
        line2  = "Seizure monitor ON";
        line3  = bleStatus;
        invert = false;
        break;

      default:
        line1 = "NORMAL";
        line2 = "Monitoring...";
        line3 = bleStatus;
        break;
    }
  }

  // Intercept manageBuzzer() alert transition
  // We detect "alert sent" by watching for mode==SEIZURE_MODE + !buzzerActive
  // + fallConfirmedTime set + consecutiveWindows >= SEIZURE_WINDOWS_NEEDED
  if (!buzzerActive && mode == SEIZURE_MODE && fallConfirmedTime > 0
      && consecutiveWindows >= SEIZURE_WINDOWS_NEEDED && !seizureDetected) {
    // This state means buzzer completed without button press → alert was sent
    alertSent = true;
  }
  // Reset alertSent on full system reset (fullSystemReset sets mode=FALL_MODE)
  if (mode == FALL_MODE) alertSent = false;

  oledUpdate(line1, line2, line3, invert);
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
