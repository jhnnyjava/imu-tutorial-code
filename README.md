# MPU6050 IMU Sensor Tutorial for CanSat Projects

![MPU6050](https://img.shields.io/badge/Sensor-MPU6050-blue) ![Platform](https://img.shields.io/badge/Platform-ESP32-green) ![Framework](https://img.shields.io/badge/Framework-Arduino-teal)

This project demonstrates how to use the **MPU6050 6-axis Inertial Measurement Unit (IMU)** in CanSat applications. The MPU6050 combines a 3-axis accelerometer and 3-axis gyroscope, making it perfect for tracking motion, orientation, and detecting mission events like launch, apogee, and landing.

---

## 📋 Table of Contents

1. [What is the MPU6050?](#what-is-the-mpu6050)
2. [Hardware Setup](#hardware-setup)
3. [Software Installation](#software-installation)
4. [How the Code Works](#how-the-code-works)
5. [Understanding the Output](#understanding-the-output)
6. [Testing Your Setup](#testing-your-setup)
7. [Advanced Features](#advanced-features)
8. [Troubleshooting](#troubleshooting)
9. [CanSat Mission Integration](#cansat-mission-integration)
10. [References](#references)

---

## 🔬 What is the MPU6050?

The **MPU6050** is a 6-axis Motion Processing Unit that combines:

- **3-axis Accelerometer**: Measures linear acceleration (X, Y, Z) in m/s²
  - Detects gravity, movement, vibration, and impact forces
  - Useful for detecting launch, apogee, and landing events

- **3-axis Gyroscope**: Measures angular velocity (X, Y, Z) in rad/s or °/s
  - Detects rotation and orientation changes
  - Helps stabilize orientation or control systems

- **Temperature Sensor**: Measures chip temperature in °C
  - Useful for environmental monitoring and health checks

### Key Specifications

| Feature | Value |
|---------|-------|
| Communication | I2C (address 0x68 or 0x69) |
| Supply Voltage | 3.3V or 5V |
| Accelerometer Range | ±2g, ±4g, ±8g, ±16g |
| Gyroscope Range | ±250°/s, ±500°/s, ±1000°/s, ±2000°/s |
| Digital Low-Pass Filter | 5Hz to 260Hz |
| Power Consumption | ~3.9mA (typical) |

---

## 🔌 Hardware Setup

### Components Needed

- **ESP32** development board (or Arduino Uno, Nano, etc.)
- **MPU6050** sensor module (GY-521 breakout board)
- **Jumper wires** (4 wires minimum)
- **Breadboard** (optional)
- **USB cable** for programming

### Wiring Diagram

Connect the MPU6050 to your ESP32 as follows:

```
MPU6050 Pin  →  ESP32 Pin
━━━━━━━━━━━━━━━━━━━━━━━
VCC          →  3.3V (or 5V if module has voltage regulator)
GND          →  GND
SDA          →  GPIO 21 (default I2C SDA)
SCL          →  GPIO 22 (default I2C SCL)
```

**For Arduino Uno/Nano:**
```
MPU6050 Pin  →  Arduino Pin
━━━━━━━━━━━━━━━━━━━━━━━━━
VCC          →  5V
GND          →  GND
SDA          →  A4
SCL          →  A5
```

### Notes
- Most MPU6050 breakout boards have built-in voltage regulators and can accept 5V
- The AD0 pin can be left unconnected (defaults to I2C address 0x68) or tied to VCC for 0x69
- Optional: Add 4.7kΩ pull-up resistors to SDA and SCL if experiencing communication issues

### Visual Reference

```
    ┌─────────────────┐
    │     MPU6050     │
    │    (Top View)   │
    │                 │
    │  VCC  GND      │
    │  [●]  [●]      │
    │                 │
    │  SCL  SDA  AD0 │
    │  [●]  [●]  [●] │
    └─────────────────┘
         │    │    │
         │    │    └── (Optional: Address select)
         │    └──────── To GPIO 21 (SDA)
         └───────────── To GPIO 22 (SCL)
```

---

## 💾 Software Installation

### Step 1: Install PlatformIO

This project uses **PlatformIO** for easy dependency management.

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
3. Restart VS Code

### Step 2: Open the Project

1. Open this folder in VS Code
2. PlatformIO will automatically detect `platformio.ini`
3. Install dependencies by clicking "Build" or running:
   ```bash
   pio lib install
   ```

### Step 3: Upload to Your Board

1. Connect your ESP32 via USB
2. Click the **Upload** button (→) in the PlatformIO toolbar, or run:
   ```bash
   pio run --target upload
   ```
3. Open the **Serial Monitor** (plug icon) at 115200 baud

---

## 🧠 How the Code Works

### Code Structure Overview

```
main.cpp
├── Includes & Configuration
├── Global Objects (mpu sensor, timing variables)
├── setup()
│   ├── Initialize Serial communication
│   ├── Initialize I2C bus
│   ├── Initialize MPU6050 sensor
│   └── Configure sensor ranges and filters
└── loop()
    ├── Non-blocking timing check
    ├── Read sensor data (accel, gyro, temp)
    └── Display data (readable or JSON format)
```

---

### Line-by-Line Explanation

#### **1. Library Includes**

```cpp
#include <Arduino.h>           // Core Arduino functions
#include <Wire.h>              // I2C communication library
#include <Adafruit_MPU6050.h>  // MPU6050 driver library
#include <Adafruit_Sensor.h>   // Unified sensor interface
```

- **Arduino.h**: Provides core functions like `Serial`, `delay()`, `millis()`
- **Wire.h**: Implements I2C protocol for communicating with the MPU6050
- **Adafruit_MPU6050.h**: High-level library that simplifies sensor initialization and reading
- **Adafruit_Sensor.h**: Defines `sensors_event_t` structure for consistent data format across sensors

#### **2. Sensor Object Creation**

```cpp
Adafruit_MPU6050 mpu;
```

Creates an instance of the MPU6050 sensor object. This object handles all communication with the physical sensor chip

---

### **setup() Function**

The `setup()` function runs **once** when the board powers on or resets.

#### **Initialize Serial Communication**

```cpp
Serial.begin(115200);
```

- Opens serial port at 115200 baud (bits per second)
- This allows us to see debug messages and sensor data in the Serial Monitor

#### **Initialize I2C Bus**

```cpp
Wire.begin();
```

- Starts the I2C communication protocol
- On ESP32: SDA = GPIO 21, SCL = GPIO 22 (default pins)
- On Arduino Uno: SDA = A4, SCL = A5

#### **Initialize MPU6050 Sensor**

```cpp
if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip!");
    // Print troubleshooting tips
    while (1) { delay(10); }  // Halt execution
}
```

**What `mpu.begin()` does:**
1. Probes the I2C bus for a device at address 0x68 (or 0x69)
2. Wakes up the MPU6050 (it starts in sleep mode by default)
3. Performs basic reset and initialization
4. Returns `true` if successful, `false` if the sensor isn't found

**Why halt on failure?**
- If the sensor isn't detected, there's a hardware issue (wiring, power, etc.)
- Halting prevents the code from continuing with invalid data
- Forces you to fix the problem before proceeding

#### **Configure Sensor Ranges**

```cpp
mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu.setGyroRange(MPU6050_RANGE_500_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
```

**Accelerometer Range (±8g):**
- **Trade-off**: Wider range = less sensitivity but handles bigger forces
- **Why ±8g for CanSat?**
  - Rocket launches can produce 3-5g of acceleration
  - Landing impacts can spike to 10-20g momentarily
  - ±8g is a good middle ground (±16g if expecting very hard impacts)

**Gyroscope Range (±500°/s):**
- **Trade-off**: Wider range = less precision for slow rotations
- **Why ±500°/s for CanSat?**
  - Spinning during descent is typically < 200°/s
  - Tumbling can reach 500°/s
  - ±500°/s captures normal flight dynamics without being overly sensitive

**Filter Bandwidth (21Hz):**
- **Low-pass filter**: Removes high-frequency noise (vibrations, electrical interference)
- **Why 21Hz?**
  - CanSat dynamics are slow (< 10Hz)
  - Filters out motor vibrations and wind gusts
  - Higher bandwidth (e.g., 260Hz) is too noisy; lower (e.g., 5Hz) is too sluggish

**Available Options:**
```cpp
// Accelerometer: 2G, 4G, 8G, 16G
// Gyroscope: 250, 500, 1000, 2000 deg/s
// Filter: 260Hz, 184Hz, 94Hz, 44Hz, 21Hz, 10Hz, 5Hz
```

---

### **loop() Function**

The `loop()` function runs **continuously** after `setup()` completes.

#### **Reading Sensor Data**

```cpp
sensors_event_t accel, gyro, temp;
mpu.getEvent(&accel, &gyro, &temp);
```

**`sensors_event_t` structure:**
- Part of Adafruit's Unified Sensor library
- Consistent format across different sensors (MPU6050, BMP280, etc.)
- Contains fields like `.acceleration.x`, `.gyro.z`, `.temperature`

**`mpu.getEvent(&accel, &gyro, &temp)`:**
- Reads 14 bytes from MPU6050 registers via I2C
- Converts raw ADC values to physical units (m/s², rad/s, °C)
- Populates all three event structures in one efficient call

**Data structure:**
```cpp
accel.acceleration.x  // X-axis acceleration (m/s²)
accel.acceleration.y  // Y-axis acceleration (m/s²)
accel.acceleration.z  // Z-axis acceleration (m/s²)

gyro.gyro.x  // X-axis rotation (rad/s)
gyro.gyro.y  // Y-axis rotation (rad/s)
gyro.gyro.z  // Z-axis rotation (rad/s)

temp.temperature  // Chip temperature (°C)
```

#### **Displaying Data**

```cpp
Serial.print("Acceleration X: ");
Serial.print(accel.acceleration.x);
// ... more prints ...
```

- Uses `Serial.print()` and `Serial.println()` to display data
- Data is formatted with labels so it's easy to read
- Output appears in the Serial Monitor
  
```cpp
delay(500);  // Wait 0.5 seconds
```

- Pauses for 500 milliseconds (0.5 seconds) between readings
- This prevents flooding the Serial Monitor with too much data
- You can adjust this value: `100` = faster (10 readings/sec), `1000` = slower (1 reading/sec)

---

## 📊 Understanding the Output

### Accelerometer Data

**Units:** m/s² (meters per second squared)

**What it measures:**
- Linear acceleration in three axes (X, Y, Z)
- Includes gravity (9.81 m/s² on Earth)

**Example at rest (sensor lying flat):**
```
X: 0.00 m/s²  (no acceleration)
Y: 0.00 m/s²  (no acceleration)
Z: 9.81 m/s²  (gravity pointing up)
```

**Interpretation:**
- **During launch**: Z-axis will spike to 30-50 m/s² (3-5g)
- **Free fall/apogee**: All axes near 0 m/s² (weightlessness)
- **Landing**: Sudden spike in Z-axis (impact detection)

**Tips:**
- If Z ≠ 9.81 at rest, your sensor may need calibration
- Integrate acceleration over time to estimate velocity
- Integrate velocity over time to estimate position (drift accumulates!)

---

### Gyroscope Data

**Units:** rad/s (radians per second) or °/s (degrees per second)

**What it measures:**
- Angular velocity (rotation speed) around three axes

**Conversion:** 1 rad/s = 57.2958 °/s

**Example at rest:**
```
X: 0.00 rad/s (0.0 °/s)   (no rotation)
Y: 0.00 rad/s (0.0 °/s)   (no rotation)
Z: 0.00 rad/s (0.0 °/s)   (no rotation)
```

**Interpretation:**
- **During stable flight**: Near 0 (minor wobble < 0.5 rad/s)
- **Spinning**: Z-axis will show rotation during descent
- **Tumbling**: All axes show rapid changes

**Tips:**
- Gyros have **drift** (slow accumulation of error over time)
- Combine with accelerometer for sensor fusion (Madgwick/Mahony filter)
- Use gyro to detect orientation changes or stabilize cameras

---

### Temperature Data

**Units:** °C (degrees Celsius)

**What it measures:**
- Internal chip temperature (not ambient air temperature)

**Typical range:** 20-40°C

**Use cases:**
- Health monitoring (chip overheating?)
- Environmental correlation (cold at high altitude)
- Not accurate for air temperature (use DHT22 or BMP280 instead)

---

## 🧪 Testing Your Setup

### Step 1: Upload and Monitor

1. Upload the code to your board
2. Open Serial Monitor at **115200 baud**
3. You should see:
   ```
   === MPU6050 CanSat Tutorial ===
   Initializing sensor...
   
   ✅ MPU6050 initialized successfully!
   
   Configuration:
     - Accelerometer: ±8g
     - Gyroscope: ±500°/s
     - Filter: 21Hz bandwidth
   
   Starting measurements...
   ```

### Step 2: Test Accelerometer

**Test 1: At Rest**
- Place sensor flat on table
- Z-axis should read ~9.81 m/s² (gravity)
- X and Y should be near 0

**Test 2: Rotate 90°**
- Tilt sensor on its side
- Gravity should shift to X or Y axis
- Observe how values change

**Test 3: Shake**
- Shake the sensor
- All axes should show spikes
- Simulates launch vibrations

### Step 3: Test Gyroscope

**Test 1: At Rest**
- Keep sensor still
- All gyro axes should be near 0
- Small drift (< 0.01 rad/s) is normal

**Test 2: Rotate Slowly**
- Rotate sensor around Z-axis (spin like a record)
- Gyro Z should show positive or negative values
- Stop rotating: should return to ~0

**Test 3: Tumble**
- Rotate rapidly in all directions
- All gyro axes should respond
- Simulates tumbling during descent

### Step 4: Temperature

- Chip temperature should be stable (~25-30°C indoors)
- Pinch the chip with your fingers: temperature should rise slightly
- Not accurate for ambient temperature measurement

---

## 🚀 Advanced Features

### 1. Change Sample Rate

To read the sensor faster or slower, change the delay value in `loop()`:

```cpp
delay(500);  // Current: 0.5 seconds (2 readings/second)
```

Change to:
- `delay(100);` = 10 readings/second (faster)
- `delay(1000);` = 1 reading/second (slower)

---

### 2. MQTT Integration

**Goal:** Send IMU data wirelessly to a base station

**Step 1:** Add MQTT library to `platformio.ini`:
```ini
lib_deps = 
    ...
    knolleary/PubSubClient@^2.8
```

**Step 2:** In `main.cpp`, add WiFi and MQTT setup:
```cpp
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    // ... existing setup ...
    
    WiFi.begin("YOUR_SSID", "YOUR_PASSWORD");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    client.setServer("mqtt_broker_ip", 1883);
}
```

**Step 3:** Publish data in `loop()`:
```cpp
void loop() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Create simple message
    String message = String(accel.acceleration.x) + "," + 
                     String(accel.acceleration.y) + "," + 
                     String(accel.acceleration.z);
    
    if (client.connected()) {
        client.publish("cansat/telemetry", message.c_str());
    }
    
    delay(500);
}
```

---

### 3. Calibration

**Problem:** Sensor readings may have bias (e.g., Z-axis reads 9.5 instead of 9.81)

**Solution:** Measure offsets and subtract in software

**Calibration procedure:**
1. Place sensor flat and still
2. Record 100 samples of accel and gyro
3. Calculate average (this is your bias)
4. Subtract bias from all future readings

**Example:**
```cpp
float accelBiasX = 0.15;  // Measured during calibration
float accelBiasY = -0.08;
float accelBiasZ = -0.30;

// In loop():
float correctedX = accel.acceleration.x - accelBiasX;
float correctedY = accel.acceleration.y - accelBiasY;
float correctedZ = accel.acceleration.z - accelBiasZ;
```

---

### 4. Sensor Fusion (Orientation Estimation)

**Goal:** Combine accelerometer + gyroscope to estimate 3D orientation (roll, pitch, yaw)

**Why?** 
- Accelerometer alone: affected by linear motion
- Gyroscope alone: drifts over time
- Fusion: best of both worlds

**Popular algorithms:**
- **Complementary Filter**: Simple, fast (good for beginners)
- **Madgwick Filter**: More accurate, moderate complexity
- **Kalman Filter**: Most accurate, complex

**Library:** [Madgwick AHRS](https://github.com/arduino-libraries/MadgwickAHRS)

---

### 5. Event Detection

**Detect key mission events using IMU data:**

```cpp
// Launch detection
if (accel.acceleration.z > 20.0) {  // > 2g
    Serial.println("🚀 LAUNCH DETECTED!");
}

// Apogee detection (near weightlessness)
float totalAccel = sqrt(
    accel.acceleration.x * accel.acceleration.x +
    accel.acceleration.y * accel.acceleration.y +
    accel.acceleration.z * accel.acceleration.z
);
if (totalAccel < 2.0) {  // < 0.2g
    Serial.println("🎯 APOGEE!");
}

// Landing detection
if (accel.acceleration.z > 50.0) {  // Hard impact
    Serial.println("🛬 LANDING!");
}
```

---

## 🔧 Troubleshooting

### Problem: "Failed to find MPU6050 chip"

**Possible causes:**
1. **Wiring issue**: Check connections (SDA, SCL, VCC, GND)
2. **Wrong I2C address**: Try 0x69 if AD0 is pulled high
3. **Power issue**: Ensure 3.3V or 5V supply is stable
4. **I2C pull-ups**: Add 4.7kΩ resistors if using long wires

**Debug steps:**
1. Run an I2C scanner sketch to detect devices
2. Use a multimeter to check voltage on VCC pin
3. Try a different MPU6050 module (could be defective)

---

### Problem: Accelerometer reads weird values at rest

**Expected:** Z ≈ 9.81 m/s², X ≈ 0, Y ≈ 0  
**Actual:** Z = 8.5, X = 1.2, Y = -0.5

**Solution:** Calibration needed (see [Calibration](#3-calibration))

---

### Problem: Gyroscope drifts over time

**Cause:** Gyroscopes have inherent bias and drift

**Solutions:**
1. Subtract bias measured during calibration
2. Use sensor fusion (combine with accelerometer)
3. Reset integration periodically using known states

---

### Problem: Noisy data / erratic readings

**Solutions:**
1. Lower filter bandwidth: `mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);`
2. Average multiple samples in software
3. Check for loose wiring (vibrations cause intermittent connections)
4. Shield I2C wires from electromagnetic interference

---

### Problem: Serial Monitor shows garbage characters

**Cause:** Baud rate mismatch

**Solution:** Ensure Serial Monitor is set to **115200 baud**

---

## 🛰️ CanSat Mission Integration

### Typical CanSat Architecture

```
┌─────────────────────────────────────┐
│         CanSat System               │
├─────────────────────────────────────┤
│  ESP32 Microcontroller              │
│   ├── MPU6050 (this project)        │
│   ├── BMP280 (pressure/altitude)    │
│   ├── DHT22 (temperature/humidity)  │
│   ├── GPS Module                    │
│   ├── SD Card (data logging)        │
│   └── LoRa/WiFi (telemetry)         │
└─────────────────────────────────────┘
```

### Mission Phases & IMU Usage

| Phase | IMU Data | Action |
|-------|----------|--------|
| **Pre-Launch** | Monitor for stillness | Calibrate sensors, await launch |
| **Launch** | High Z-axis accel (30-50 m/s²) | Detect launch, start logging |
| **Ascent** | Decreasing accel | Track acceleration profile |
| **Apogee** | Near 0 accel (free fall) | Trigger parachute deployment |
| **Descent** | Stable ~9.81 m/s² Z | Monitor spin rate (gyro Z) |
| **Landing** | Impact spike (>50 m/s²) | Stop logging, send final telemetry |

### Integration Example: Parachute Deployment

```cpp
bool apogeeDetected = false;

void loop() {
    // ... read MPU6050 ...
    
    float totalAccel = sqrt(
        accel.acceleration.x * accel.acceleration.x +
        accel.acceleration.y * accel.acceleration.y +
        accel.acceleration.z * accel.acceleration.z
    );
    
    if (!apogeeDetected && totalAccel < 2.0) {  // Weightlessness
        apogeeDetected = true;
        deployParachute();  // Trigger servo/motor
        Serial.println("🪂 Parachute deployed!");
    }
}
```

### Data Fusion: IMU + Barometer

Combine MPU6050 acceleration with BMP280 altitude for robust event detection:

```cpp
// Apogee: altitude stops increasing AND low acceleration
if (altitude < lastAltitude && totalAccel < 2.0) {
    // Confirmed apogee
}
```

---

## 📚 References

### Documentation
- [MPU6050 Datasheet (InvenSense)](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)
- [PlatformIO Documentation](https://docs.platformio.org/)

### Tutorials
- [I2C Protocol Explained](https://learn.sparkfun.com/tutorials/i2c)
- [Sensor Fusion Basics](https://www.youtube.com/watch?v=T9jXoG0QYIA)
- [Kalman Filter Tutorial](https://www.kalmanfilter.net/)

### Libraries
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [ArduinoJson](https://arduinojson.org/)
- [PubSubClient (MQTT)](https://pubsubclient.knolleary.net/)

---

## 🤝 Contributing

Found a bug? Have a suggestion? Open an issue or submit a pull request!

---

## 📄 License

This project is open-source and available under the MIT License.

---

## 🎓 Learning Goals

By completing this tutorial, you should be able to:

✅ Understand how the MPU6050 IMU works  
✅ Wire and initialize the sensor via I2C  
✅ Read accelerometer, gyroscope, and temperature data  
✅ Interpret sensor data in the context of CanSat missions  
✅ Implement non-blocking sensor sampling  
✅ Integrate IMU with other sensors (BMP280, GPS)  
✅ Send telemetry via MQTT or LoRa  
✅ Detect mission events (launch, apogee, landing)  

---

## 🚀 Next Steps

1. **Test your setup**: Upload the code and verify readings
2. **Experiment**: Shake, rotate, and tilt the sensor
3. **Calibrate**: Measure and correct bias errors
4. **Integrate**: Combine with other sensors (BMP280, GPS)
5. **Deploy**: Add WiFi/MQTT for wireless telemetry
6. **Fly**: Test in a model rocket or CanSat competition!

**Good luck with your CanSat mission! 🛰️**

---

*Made with ❤️ for CanSat enthusiasts*
