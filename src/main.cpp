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

Adafruit_MPU6050 mpu;  // Create sensor object

void setup() {
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  Serial.println("MPU6050 Test");
  
  // Start I2C communication
  Wire.begin();
  
  // Try to initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  // Configure the sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);      // ±8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);           // ±500°/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);        // 21 Hz filter
  
  Serial.println("");
  delay(100);
}

void loop() {
  // Create structures to hold sensor data
  sensors_event_t accel, gyro, temp;
  
  // Read sensor data
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Print accelerometer data (m/s²)
  Serial.print("Acceleration X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s²");
  
  // Print gyroscope data (rad/s)
  Serial.print("Rotation X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(", Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(", Z: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" rad/s");
  
  // Print temperature (°C)
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");
  
  Serial.println("");
  delay(500);  // Wait 0.5 seconds before next reading
}