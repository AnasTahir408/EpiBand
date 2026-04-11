#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Seizure detection thresholds - TUNE THESE
const float SEIZURE_SHAKE_THRESHOLD = 1.5;   // g - Lower = more sensitive
const float SEIZURE_GYRO_THRESHOLD = 100.0;  // deg/s - Lower = more sensitive
const unsigned long SEIZURE_DETECTION_WINDOW = 1000; // ms
const int SEIZURE_MIN_SHAKES = 2;            // Shakes per second
const unsigned long SEIZURE_CONFIRM_TIME = 3000; // ms - 3 seconds

// Variables
unsigned long windowStartTime = 0;
int shakeCount = 0;
int consecutiveSeizureWindows = 0;
bool lastWasShake = false;
float lastAccelMagnitude = 1.0;
unsigned long lastShakeTime = 0;
bool seizureDetected = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("=================================");
  Serial.println("SEIZURE DETECTION TESTING MODE");
  Serial.println("=================================");
  
  if (! mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("✓ Ready!  Start shaking the device.. .");
  Serial.println("=================================\n");
  
  windowStartTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Acceleration (in g)
  float accelX = a.acceleration.x / 9.81;
  float accelY = a.acceleration.y / 9.81;
  float accelZ = a.acceleration. z / 9.81;
  float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  
  // Gyroscope (in deg/s)
  float gyroX = g.gyro.x * 180.0 / PI;
  float gyroY = g.gyro.y * 180.0 / PI;
  float gyroZ = g. gyro.z * 180.0 / PI;
  float gyroMagnitude = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
  
  // Detect shaking
  float accelChange = abs(accelMagnitude - lastAccelMagnitude);
  lastAccelMagnitude = accelMagnitude;
  
  // Check for shake (acceleration change OR gyro movement)
  if ((accelChange > SEIZURE_SHAKE_THRESHOLD) || (gyroMagnitude > SEIZURE_GYRO_THRESHOLD)) {
    if (! lastWasShake) {
      shakeCount++;
      lastWasShake = true;
      lastShakeTime = millis();
      
      Serial.print("📳 Shake #");
      Serial.print(shakeCount);
      Serial.print(" | Accel Δ: ");
      Serial.print(accelChange, 2);
      Serial.print("g | Gyro:  ");
      Serial.print(gyroMagnitude, 1);
      Serial.println("°/s");
    }
  } else {
    lastWasShake = false;
  }
  
  // Check every second
  if (millis() - windowStartTime >= SEIZURE_DETECTION_WINDOW) {
    Serial.print("⏱️  Window:  ");
    Serial.print(shakeCount);
    Serial.print(" shakes | ");
    
    if (shakeCount >= SEIZURE_MIN_SHAKES) {
      consecutiveSeizureWindows++;
      Serial.print("⚠️  SEIZURE ACTIVITY!  (");
      Serial.print(consecutiveSeizureWindows);
      Serial.println(" sec)");
      
      // Confirm seizure after continuous shaking
      if (consecutiveSeizureWindows >= (SEIZURE_CONFIRM_TIME / SEIZURE_DETECTION_WINDOW)) {
        if (! seizureDetected) {
          Serial.println();
          Serial.println("🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");
          Serial.println("🚨 SEIZURE DETECTED!!!     🚨");
          Serial.println("🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");
          Serial.println();
          seizureDetected = true;
        }
      }
    } else {
      consecutiveSeizureWindows = 0;
      Serial.println("✓ Normal");
      seizureDetected = false;
    }
    
    windowStartTime = millis();
    shakeCount = 0;
  }
  
  // Check if seizure stopped
  if (seizureDetected && (millis() - lastShakeTime > 5000)) {
    Serial.println("\nℹ️  Seizure stopped\n");
    seizureDetected = false;
    consecutiveSeizureWindows = 0;
  }
  
  delay(10);
}
