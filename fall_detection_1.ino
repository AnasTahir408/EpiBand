#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU6050 object
Adafruit_MPU6050 mpu;

// Fall detection thresholds
const float FREE_FALL_THRESHOLD = 0.5;      // g (gravitational force)
const float IMPACT_THRESHOLD = 3.0;          // g
const float TILT_THRESHOLD = 45.0;           // degrees
const unsigned long INACTIVITY_TIME = 3000;  // ms (3 seconds)
const unsigned long FREE_FALL_MIN_TIME = 100; // ms

// State machine states
enum FallState {
  NORMAL,
  FREE_FALL_DETECTED,
  IMPACT_DETECTED,
  FALL_CONFIRMED
};

FallState currentState = NORMAL;

// Timing variables
unsigned long freeFallStartTime = 0;
unsigned long impactTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial to be ready
  
  Serial.println("=================================");
  Serial.println("ESP32 Fall Detection System");
  Serial.println("=================================");
  Serial.println("Initializing MPU6050...");
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip");
    Serial.println("Check wiring:");
    Serial.println("  MPU6050 VCC → ESP32 3.3V");
    Serial.println("  MPU6050 GND → ESP32 GND");
    Serial.println("  MPU6050 SCL → ESP32 GPIO22");
    Serial.println("  MPU6050 SDA → ESP32 GPIO21");
    while (1) {
      delay(10);
    }
  }
  Serial.println("✓ MPU6050 Found!");
  
  // Set sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("✓ Sensor configured");
  Serial.println("✓ Fall detection system ready!");
  Serial.println("=================================");
  Serial.println();
  delay(1000);
}

void loop() {
  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Extract acceleration values (already in m/s², convert to g)
  float accelX = a.acceleration.x / 9.81;
  float accelY = a.acceleration.y / 9.81;
  float accelZ = a.acceleration.z / 9.81;
  
  // Calculate total acceleration magnitude
  float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  
  // Calculate tilt angle (pitch and roll)
  float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float roll = atan2(-accelX, accelZ) * 180.0 / PI;
  float tiltAngle = max(abs(pitch), abs(roll));
  
  // Fall detection state machine
  switch (currentState) {
    case NORMAL:
      // Check for free fall (sudden drop in acceleration)
      if (accelMagnitude < FREE_FALL_THRESHOLD) {
        freeFallStartTime = millis();
        currentState = FREE_FALL_DETECTED;
        Serial.println("⚠️  FREE FALL DETECTED!");
        Serial.print("    Time: ");
        Serial.print(millis());
        Serial.println(" ms");
      }
      break;
      
    case FREE_FALL_DETECTED:
      // Check if free fall continues
      if (accelMagnitude < FREE_FALL_THRESHOLD) {
        // Still in free fall - continue monitoring
        if (millis() - freeFallStartTime > 2000) {
          // Free fall too long - reset (might be sensor error)
          currentState = NORMAL;
          Serial.println("⚠️  Free fall timeout - resetting to NORMAL");
        }
      } else if (accelMagnitude > IMPACT_THRESHOLD) {
        // Impact detected after free fall
        if (millis() - freeFallStartTime >= FREE_FALL_MIN_TIME) {
          impactTime = millis();
          currentState = IMPACT_DETECTED;
          Serial.println("💥 IMPACT DETECTED!");
          Serial.print("    Magnitude: ");
          Serial.print(accelMagnitude, 2);
          Serial.println(" g");
          Serial.print("    Free fall duration: ");
          Serial.print(millis() - freeFallStartTime);
          Serial.println(" ms");
          Serial.println("    Monitoring for inactivity...");
        } else {
          // Free fall was too short
          currentState = NORMAL;
          Serial.println("⚠️  Free fall too brief - resetting to NORMAL");
        }
      } else {
        // Acceleration returned to normal without impact
        currentState = NORMAL;
        Serial.println("⚠️  Acceleration normalized - resetting to NORMAL");
      }
      break;
      
    case IMPACT_DETECTED:
      // Check for abnormal tilt and inactivity
      if (millis() - impactTime > INACTIVITY_TIME) {
        // Check if person is still lying down (tilted)
        if (tiltAngle > TILT_THRESHOLD) {
          currentState = FALL_CONFIRMED;
          
          // FALL DETECTED - Print alert to Serial Monitor
          Serial.println();
          Serial.println("🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");
          Serial.println("🚨                           🚨");
          Serial.println("🚨   FALL DETECTED!!!        🚨");
          Serial.println("🚨                           🚨");
          Serial.println("🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");
          Serial.println();
          Serial.print("Fall confirmed at: ");
          Serial.print(millis());
          Serial.println(" ms");
          Serial.print("Final tilt angle: ");
          Serial.print(tiltAngle, 1);
          Serial.println("°");
          Serial.println("=================================");
          Serial.println();
          
          // Wait before resetting to avoid multiple alerts
          delay(5000);
          currentState = NORMAL;
          Serial.println("System reset - monitoring resumed");
          Serial.println();
        } else {
          // Person got up - false alarm
          currentState = NORMAL;
          Serial.println("✅ Person recovered - No fall detected");
          Serial.print("    Tilt angle: ");
          Serial.print(tiltAngle, 1);
          Serial.println("° (below threshold)");
          Serial.println();
        }
      }
      break;
  }
  
  // Debug output - print sensor readings every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    
    Serial.print("Accel: ");
    Serial.print(accelMagnitude, 2);
    Serial.print(" g | Tilt: ");
    Serial.print(tiltAngle, 1);
    Serial.print("° | State: ");
    Serial.println(getStateName(currentState));
  }
  
  delay(10); // Sample at ~100Hz
}

String getStateName(FallState state) {
  switch (state) {
    case NORMAL: return "NORMAL";
    case FREE_FALL_DETECTED: return "FREE_FALL";
    case IMPACT_DETECTED: return "IMPACT";
    case FALL_CONFIRMED: return "FALL_CONFIRMED";
    default: return "UNKNOWN";
  }
}