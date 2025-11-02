/*
 * MPU6050 IMU Sensor Tutorial for CanSat Projects
 * 
 * This simple code shows how to:
 * 1. Initialize the MPU6050 sensor
 * 2. Read accelerometer, gyroscope, and temperature data
 * 3. Display the data in Serial Monitor
 * 
 * Hardware connections:
 * - VCC → 3.3V
 * - GND → GND
 * - SDA → GPIO 21 (ESP32) or A4 (Arduino)
 * - SCL → GPIO 22 (ESP32) or A5 (Arduino)
 */

#include <Arduino.h>           // Core Arduino functions
#include <Wire.h>              // I2C communication library
#include <Adafruit_MPU6050.h>  // MPU6050 sensor library
#include <Adafruit_Sensor.h>   // Unified sensor library

// Networking and telemetry
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

Adafruit_MPU6050 mpu;  // Create sensor object

// --- Configuration (replace with your values) ---
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

const char* MQTT_SERVER = "mqtt.example.com"; // broker host or IP
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC = "imu/data";
const char* MQTT_CLIENT_ID = "imu_esp32_client";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
    if (millis() - start > 20000) { // 20s timeout
      Serial.println("\nWiFi connect timed out, retrying...");
      start = millis();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  if (mqttClient.connected()) return;
  Serial.print("Connecting to MQTT...");
  while (!mqttClient.connected()) {
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println(" connected");
      // Optionally subscribe here
    } else {
      Serial.print(" failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("; retrying in 2s");
      delay(2000);
    }
  }
}

void setup() {
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  Serial.println("MPU6050 + MQTT Test");

  // Start I2C communication
  Wire.begin();

  // Try to initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  // Configure the sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);      // ±8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);           // ±500°/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);        // 21 Hz filter

  // Setup WiFi and MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  connectWiFi();
  reconnectMQTT();

  delay(100);
}

void loop() {
  // Keep network services alive
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Create structures to hold sensor data
  sensors_event_t accel, gyro, temp;

  // Read sensor data
  mpu.getEvent(&accel, &gyro, &temp);

  // Build JSON payload
  StaticJsonDocument<256> doc;
  doc["accel"]["x"] = accel.acceleration.x;
  doc["accel"]["y"] = accel.acceleration.y;
  doc["accel"]["z"] = accel.acceleration.z;
  doc["gyro"]["x"] = gyro.gyro.x;
  doc["gyro"]["y"] = gyro.gyro.y;
  doc["gyro"]["z"] = gyro.gyro.z;
  doc["temp"] = temp.temperature;
  doc["ts"] = millis();

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  // Publish JSON to MQTT (if connected)
  if (mqttClient.connected()) {
    bool ok = mqttClient.publish(MQTT_TOPIC, buffer);
    Serial.print(ok ? "Published: " : "Publish failed: ");
    Serial.println(buffer);
  } else {
    Serial.println("MQTT not connected, skipping publish");
  }

  delay(500);  // Wait 0.5 seconds before next reading
}