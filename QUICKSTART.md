# Quick Start Guide

## 🎯 Getting Started in 5 Minutes

### Option 1: Using PlatformIO (Recommended)

1. **Open the project in VS Code**
   - Launch VS Code
   - File → Open Folder
   - Select `imu tutorial code` folder

2. **Let PlatformIO install dependencies**
   - PlatformIO will automatically detect `platformio.ini`
   - Wait for library installation to complete (bottom toolbar shows progress)

3. **Connect your ESP32**
   - Plug ESP32 into USB port
   - Wait for drivers to install (Windows may take a minute)

4. **Upload the code**
   - Click the **→** (Upload) button in the PlatformIO toolbar (bottom of VS Code)
   - Or: Press `Ctrl+Alt+U`
   - Or: Click PlatformIO icon → PROJECT TASKS → env:esp32dev → General → Upload

5. **Open Serial Monitor**
   - Click the **🔌** (Serial Monitor) icon in PlatformIO toolbar
   - Or: Press `Ctrl+Alt+S`
   - Or: PlatformIO icon → PROJECT TASKS → env:esp32dev → Monitor

### Option 2: Using Arduino IDE

If you prefer Arduino IDE over PlatformIO:

1. **Install libraries** (Tools → Manage Libraries):
   - Adafruit MPU6050
   - Adafruit Unified Sensor
   - Adafruit BusIO
   - ArduinoJson (optional, for JSON mode)

2. **Open the code**:
   - Open `src/main.cpp` in Arduino IDE
   - Rename extension to `.ino` if needed

3. **Select your board**:
   - Tools → Board → ESP32 Arduino → ESP32 Dev Module

4. **Upload**:
   - Click Upload button
   - Open Serial Monitor at 115200 baud

---

## 📺 What You Should See

After uploading, the Serial Monitor should display:

```
=== MPU6050 Test ===
MPU6050 Found!

Acceleration X: 0.12, Y: -0.05, Z: 9.83 m/s²
Rotation X: 0.001, Y: -0.002, Z: 0.000 rad/s
Temperature: 28.3 °C

Acceleration X: 0.10, Y: -0.03, Z: 9.81 m/s²
Rotation X: 0.000, Y: 0.001, Z: -0.001 rad/s
Temperature: 28.4 °C
```

### Quick Tests

**Test 1: Tilt the sensor**
- Z-axis accelerometer value should change
- When vertical, X or Y should read ~9.81 m/s²

**Test 2: Rotate the sensor**
- Gyroscope values should change
- Spin around Z-axis: gyro Z should show rotation

**Test 3: Keep it still**
- Gyro should be near 0 rad/s
- One accel axis should read ~9.81 m/s² (gravity)

---

## 🔧 Customization

### Change Sample Rate

In `src/main.cpp`, find:
```cpp
delay(500);  // milliseconds
```

Change to:
- `delay(100)` = 10Hz (faster, more data)
- `delay(1000)` = 1Hz (slower, less data)

### Enable Degrees Per Second for Gyro

Add this conversion in your print statements:
```cpp
float gyroXdeg = gyro.gyro.x * 57.2958;  // Convert rad/s to deg/s
Serial.print(gyroXdeg);
Serial.println(" deg/s");
```

### Change Sensor Ranges

In `setup()` function:

```cpp
// For higher accelerations (hard impacts):
mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

// For faster rotations (rapid spin):
mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

// For less noise filtering (faster response):
mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
```

---

## ❓ Troubleshooting

### "Failed to find MPU6050 chip"

**Check wiring:**
```
MPU6050    ESP32
VCC    →   3.3V
GND    →   GND
SDA    →   GPIO 21
SCL    →   GPIO 22
```

**Try:**
1. Re-seat all connections
2. Check voltage: VCC should have 3.3V (use multimeter)
3. Run I2C scanner (see `examples/i2c_scanner.ino`)

### Garbage in Serial Monitor

**Fix:** Set baud rate to **115200** (dropdown at bottom-right of Serial Monitor)

### Values seem wrong

**Example issues:**
- Z-axis reads 8.5 instead of 9.81 → Needs calibration
- Gyro slowly drifts → Normal, use sensor fusion
- Noisy readings → Lower filter bandwidth or average samples

See full [README.md](README.md) for detailed troubleshooting.

---

## 📖 Next Steps

1. ✅ Verify sensor works (readings change when you move it)
2. 📚 Read the full [README.md](README.md) for detailed explanations
3. 🧪 Try the examples in `examples/` folder
4. 🚀 Integrate with your CanSat system!

**Happy building! 🛰️**
