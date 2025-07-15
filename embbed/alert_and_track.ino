#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <SoftwareSerial.h>

// WiFi and Server Configuration
#define WIFI_SSID "Redmi Note 10 Pro"
#define WIFI_PASSWORD ""
#define SERVER_URL "http://192.168.89.211:3001/data"

// GPS Configuration
SoftwareSerial gpsSerial(4, 5); // RX=GPIO4, TX=GPIO5
TinyGPSPlus gps;

// MPU6050 Configuration
Adafruit_MPU6050 mpu;

// GPS Status Tracking
bool gpsFixValid = false;
unsigned long lastGpsUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Integrated System...");

  // Initialize GPS
  gpsSerial.begin(9600);
  pinMode(4, INPUT_PULLUP);
  
  // Initialize MPU6050
  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
}

void loop() {
  // Process GPS data with timeout
  unsigned long gpsStart = millis();
  while (millis() - gpsStart < 1000) { // Listen for 1 second
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      if (gps.encode(c)) {
        if (gps.location.isUpdated()) {
          gpsFixValid = true;
          lastGpsUpdate = millis();
        }
      }
    }
  }

  // Check GPS fix timeout (10 seconds)
  if (millis() - lastGpsUpdate > 10000) {
    gpsFixValid = false;
  }

  // Read MPU6050 data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate orientation (pitch and roll) in degrees
  float pitch = atan2(a.acceleration.x, 
                      sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;
  float roll  = atan2(a.acceleration.y, 
                      sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;
  
  // Determine if an accident is detected.
  // Accident is detected if either:
  //   (1) Absolute pitch > 45° OR absolute roll > 60°
  // OR
  //   (2) y-axis acceleration > 20 or x-axis acceleration < 0.
  bool accidentDetected = ((abs(pitch) < 45.0) || (abs(roll) > 60.0)) || 
                          ((a.acceleration.y > 20.0) || (a.acceleration.x < 2));

  // Debug output for orientation and accident status
  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.print("°, Roll: ");
  Serial.print(roll, 2);
  Serial.print("°, Accident Detected: ");
  Serial.println(accidentDetected ? "YES" : "NO");

  // Build JSON payload
  String jsonData = "{";
  
  // GPS Data with validity check
  jsonData += "\"gps\":{";
  if (gpsFixValid && gps.location.isValid()) {
    jsonData += "\"latitude\":" + String(gps.location.lat(), 6) + ",";
    jsonData += "\"longitude\":" + String(gps.location.lng(), 6);
  } else {
    jsonData += "\"latitude\":0.0,\"longitude\":0.0";
  }
  jsonData += "},";

  // IMU Data
  jsonData += "\"acceleration\":{";
  jsonData += "\"x\":" + String(a.acceleration.x, 2) + ",";
  jsonData += "\"y\":" + String(a.acceleration.y, 2) + ",";
  jsonData += "\"z\":" + String(a.acceleration.z, 2);
  jsonData += "},";

  // Orientation Data
  jsonData += "\"orientation\":{";
  jsonData += "\"pitch\":" + String(pitch, 2) + ",";
  jsonData += "\"roll\":" + String(roll, 2);
  jsonData += "},";

  // Accident Detection Data
  jsonData += "\"accidentDetected\":" + String(accidentDetected ? "true" : "false");
  
  jsonData += "}";
  
  // Debug output of JSON payload
  Serial.println("JSON Payload: ");
  Serial.println(jsonData);

  // Send data to server if WiFi connected
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    
    int httpCode = http.POST(jsonData);
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      Serial.println("Server response: " + response);
    } else {
      Serial.printf("HTTP Error: %d\n", httpCode);
    }
    http.end();
  } else {
    Serial.println("WiFi disconnected!");
    WiFi.reconnect();
  }

  // 3-second delay between updates
  delay(3000);
}
