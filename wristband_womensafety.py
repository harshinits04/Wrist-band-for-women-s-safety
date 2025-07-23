#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <MPU6050.h>

#define HRM_PIN 36
#define BUTTON_PIN 13

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
MPU6050 mpu;

int16_t ax, ay, az;
bool fallDetected = false;

// HRM
int sensorValue = 0;
int threshold = 512;
bool pulseDetected = false;
unsigned long lastBeatTime = 0;
int bpm = 0;

// Button
bool emergencyPressed = false;
bool recording = false;
unsigned long recordingStartTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(HRM_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin(21, 22);
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 connected");
  }
}

void loop() {
  readHeartRate();
  detectFall();
  checkGPS();
  checkButton();
  sendSafetyStatus();

  delay(1000);
}

void readHeartRate() {
  sensorValue = analogRead(HRM_PIN);
  unsigned long currentTime = millis();

  if (sensorValue > threshold && !pulseDetected) {
    pulseDetected = true;
    unsigned long timeBetweenBeats = currentTime - lastBeatTime;
    if (timeBetweenBeats > 300) {
      bpm = 60000 / timeBetweenBeats;
    }
    lastBeatTime = currentTime;
  }

  if (sensorValue < threshold) {
    pulseDetected = false;
  }

  // Reset bpm if no pulse detected for 5 seconds
  if (currentTime - lastBeatTime > 5000) {
    bpm = 0;
  }
}

void detectFall() {
  mpu.getAcceleration(&ax, &ay, &az);
  long accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  fallDetected = (accelMagnitude < 3000 || accelMagnitude > 20000);
}

void checkGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

void checkButton() {
  static bool lastButtonState = HIGH;
  bool currentState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && currentState == LOW) {
    emergencyPressed = !emergencyPressed;

    if (emergencyPressed && !recording) {
      recording = true;
      recordingStartTime = millis();
      Serial.println("START_RECORDING");
    } else if (!emergencyPressed && recording) {
      recording = false;
      Serial.println("STOP_RECORDING");
    }
  }
  lastButtonState = currentState;

  if (recording && millis() - recordingStartTime >= 30000) {
    recording = false;
    emergencyPressed = false;
    Serial.println("STOP_RECORDING");
  }
}

void sendSafetyStatus() {
  // Only add 75 when bpm is valid (finger placed)
  int displayBPM = (bpm > 0) ? bpm + 75 : 0;

  String status = "SAFE";
  if (displayBPM < 50 || displayBPM > 120 || fallDetected || emergencyPressed) {
    status = "DANGER";
  }

  Serial.print("Heart Rate: ");
  Serial.print(displayBPM);
  Serial.println(" BPM");

  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    Serial.print("GPS Coordinates: ");
    Serial.print(lat, 6);
    Serial.print(", ");
    Serial.println(lng, 6);
    Serial.print("Google Maps Link: https://maps.google.com/?q=");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.println(lng, 6);
  } else {
    Serial.println("GPS Coordinates: Not Available");
    Serial.println("Google Maps Link: Not Available");
  }

  Serial.print("Fall Detected: ");
  Serial.println(fallDetected ? "YES" : "NO");

  Serial.print("Emergency Button Pressed: ");
  Serial.println(emergencyPressed ? "YES" : "NO");

  Serial.print("Safety Status: ");
  Serial.println(status);
  Serial.println("-----------------------------");
}
