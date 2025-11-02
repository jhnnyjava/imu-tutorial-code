# Teaching Notes for MPU6050 Tutorial

## 📚 Purpose of This Project

This is a **teaching-focused** MPU6050 tutorial designed for students learning about IMU sensors in CanSat or robotics projects. The code is intentionally kept simple and well-commented to facilitate understanding.

---

## 🎯 Learning Objectives

After completing this tutorial, students should understand:

1. **What an IMU sensor is** and what it measures
2. **How I2C communication works** (SDA, SCL pins)
3. **How to initialize and configure** a sensor
4. **How to read sensor data** in a loop
5. **What the data means** (acceleration, rotation, temperature)
6. **How to use this data** in a real CanSat mission

---

## 🗂️ Project Structure

```
imu tutorial code/
├── src/
│   └── main.cpp              # Main code (80 lines, simple and clean)
├── platformio.ini            # Project configuration and libraries
├── README.md                 # Comprehensive documentation
├── QUICKSTART.md            # 5-minute setup guide
└── TEACHING_NOTES.md        # This file (for instructors)
```

---

## 👨‍🏫 Teaching Flow (Recommended)

### **Lesson 1: What is an IMU? (15 min)**

**Concepts:**
- IMU = Inertial Measurement Unit
- Measures motion and orientation
- Two main sensors: accelerometer + gyroscope

**Activities:**
- Show physical MPU6050 module
- Explain the 3 axes (X, Y, Z)
- Demonstrate by tilting/rotating the sensor

**Resources:**
- README.md → "What is the MPU6050?" section

---

### **Lesson 2: Hardware Setup (20 min)**

**Concepts:**
- I2C communication (only 2 wires for data!)
- Power supply (3.3V or 5V)
- Pin connections

**Activities:**
- Wire up the MPU6050 following the diagram
- Check connections with multimeter (optional)
- Explain what each pin does

**Common Issues:**
- Swapped SDA/SCL
- Loose connections
- Wrong voltage

**Resources:**
- README.md → "Hardware Setup" section
- QUICKSTART.md → Visual diagram

---

### **Lesson 3: Code Walkthrough (30 min)**

**Go through main.cpp line by line:**

1. **Includes** (lines 16-19)
   - Explain what each library does
   - Compare to importing modules in Python

2. **setup()** (lines 23-46)
   - Serial communication (how we see data)
   - I2C initialization
   - Sensor initialization and error handling
   - Configuration (ranges, filters)

3. **loop()** (lines 48-80)
   - Reading sensor data
   - Printing formatted output
   - delay() for timing

**Teaching Tips:**
- Run the code first, show the output
- Then go through explaining how it works
- Encourage questions at each section

**Resources:**
- README.md → "How the Code Works" section

---

### **Lesson 4: Understanding the Data (25 min)**

**Concepts:**

**Accelerometer:**
- Measures linear acceleration in m/s²
- Includes gravity (9.81 m/s²)
- At rest: one axis shows gravity

**Gyroscope:**
- Measures rotation in rad/s
- At rest: all axes near 0
- Spins/rotations show up here

**Temperature:**
- Internal chip temperature
- NOT accurate for air temperature

**Activities:**
- Place sensor flat → observe Z ≈ 9.81
- Tilt sensor → watch gravity shift
- Rotate sensor → see gyro respond
- Keep still → note gyro drift

**Resources:**
- README.md → "Understanding the Output" section

---

### **Lesson 5: Testing & Experiments (30 min)**

**Experiments:**

**Experiment 1: Gravity Test**
- Place sensor in different orientations
- Record which axis shows 9.81 m/s²
- Draw a diagram of sensor orientation

**Experiment 2: Shake Test**
- Shake sensor gently
- Observe acceleration spikes
- This simulates rocket vibrations!

**Experiment 3: Rotation Test**
- Spin sensor slowly around Z-axis
- Watch gyro Z value
- Note: positive vs negative rotation

**Experiment 4: Free Fall (Advanced)**
- Drop sensor onto soft surface (with padding!)
- Observe brief moment of ~0 acceleration
- Then impact spike when it lands

**Data Collection:**
- Have students record data in a table
- Plot graphs (Excel/Google Sheets)
- Analyze patterns

---

### **Lesson 6: CanSat Mission Application (20 min)**

**Discuss real-world uses:**

1. **Launch Detection**
   - High acceleration (>20 m/s²) = rocket launched
   - Trigger: start logging data

2. **Apogee Detection**
   - Low acceleration (~0 m/s²) = weightlessness
   - Trigger: deploy parachute

3. **Landing Detection**
   - Impact spike (>50 m/s²) = landed
   - Trigger: stop logging, send final telemetry

4. **Spin Monitoring**
   - Gyro Z axis shows rotation rate
   - Monitor stability during descent

**Activity:**
- Simulate mission phases by hand
- Students predict what sensor data would look like
- Compare predictions to actual tests

**Resources:**
- README.md → "CanSat Mission Integration" section

---

## 🔧 Common Student Issues & Solutions

### Issue: "Failed to find MPU6050 chip"

**Debugging steps:**
1. Check all 4 wires (VCC, GND, SDA, SCL)
2. Verify power: 3.3V on VCC pin
3. Swap SDA/SCL (common mistake!)
4. Try different USB port
5. Use I2C scanner code

**Teaching moment:** This is real engineering! Debugging hardware is part of the process.

---

### Issue: "Serial Monitor shows weird characters"

**Solution:** Set baud rate to 115200

**Teaching moment:** Explain baud rate = communication speed, must match on both sides

---

### Issue: "Z-axis reads 8.5 instead of 9.81"

**Solution:** This is normal! Sensors have bias.

**Teaching moment:** Introduce calibration concept, explain sensor accuracy

---

### Issue: "Gyro slowly drifts even when still"

**Solution:** This is normal! Gyroscopes drift.

**Teaching moment:** Explain sensor limitations, introduce sensor fusion

---

## 🎓 Extension Activities (For Advanced Students)

### 1. **Add Calibration**
- Measure offsets when sensor is still
- Subtract offsets from readings
- Compare before/after accuracy

### 2. **Data Logging**
- Add SD card module
- Log sensor data to CSV file
- Analyze in Excel/Python

### 3. **Event Detection**
- Add code to detect "launch" (high accel)
- Print "LAUNCH!" when detected
- Add LED that lights up on launch

### 4. **MQTT Telemetry**
- Add WiFi connection
- Send data wirelessly to computer
- Create live dashboard

### 5. **Sensor Fusion**
- Combine accel + gyro for orientation
- Implement complementary filter
- Calculate roll, pitch, yaw angles

---

## 📊 Assessment Ideas

### **Quiz Questions:**
1. What does IMU stand for?
2. What are the three axes called? (X, Y, Z)
3. What does the accelerometer measure?
4. What does the gyroscope measure?
5. Why does Z-axis read 9.81 when still?

### **Lab Practical:**
1. Wire up MPU6050 correctly
2. Upload code successfully
3. Interpret sensor readings
4. Explain one mission application

### **Project:**
- Build a simple "launch detector"
- Code must detect when acceleration exceeds threshold
- Demonstrate working system

---

## ⏱️ Suggested Timeline

| Session | Duration | Topic |
|---------|----------|-------|
| 1 | 15 min | Theory: What is an IMU? |
| 2 | 20 min | Hands-on: Wire the sensor |
| 3 | 30 min | Code walkthrough |
| 4 | 25 min | Understanding data |
| 5 | 30 min | Experiments & testing |
| 6 | 20 min | Mission applications |
| **Total** | **2h 20min** | Can split across multiple classes |

---

## 🛠️ Required Materials (Per Student/Group)

- [ ] ESP32 development board (or Arduino Uno/Nano)
- [ ] MPU6050 sensor module (GY-521)
- [ ] Breadboard
- [ ] 4x jumper wires
- [ ] USB cable
- [ ] Computer with VS Code + PlatformIO

**Optional:**
- [ ] Multimeter (for debugging)
- [ ] LED + resistor (for event detection)
- [ ] Soft padding (for drop tests)

---

## 💡 Teaching Tips

1. **Show First, Explain Later**
   - Run the working code first
   - Let students see output before explaining
   - Creates curiosity and context

2. **Hands-On Learning**
   - Students should wire their own sensors
   - Encourage experimentation
   - Mistakes are learning opportunities!

3. **Visual Aids**
   - Draw axes on whiteboard
   - Use hand gestures for rotation
   - Show videos of CanSat launches

4. **Real-World Context**
   - Always relate to CanSat mission
   - Show examples from competitions
   - Explain why each sensor matters

5. **Troubleshooting Practice**
   - Intentionally introduce an error (wrong wire)
   - Guide students through debugging
   - Builds problem-solving skills

---

## 📚 Additional Resources

### For Students:
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) (simplified version)
- [I2C Protocol Explained](https://learn.sparkfun.com/tutorials/i2c) (SparkFun tutorial)
- [CanSat Competition](https://www.cansatcompetition.com/) (examples and rules)

### For Instructors:
- [Sensor Fusion Tutorial](https://www.youtube.com/watch?v=T9jXoG0QYIA)
- [Kalman Filter Basics](https://www.kalmanfilter.net/kalman1d.html)
- [Arduino Reference](https://www.arduino.cc/reference/en/)

---

## 🤝 Facilitation Notes

### Group Work:
- Pairs work well (1 wiring, 1 coding)
- Rotate roles between lessons
- Share findings with class

### Pacing:
- Check understanding before moving on
- Allow time for questions
- Some students will be faster → have extension activities ready

### Safety:
- Warn about short circuits (VCC to GND)
- Supervise soldering if needed
- Protect sensors during drop tests

---

## ✅ Key Takeaways for Students

By the end of this tutorial, students should be able to:

✅ Explain what an IMU sensor measures  
✅ Wire an I2C sensor correctly  
✅ Read and modify Arduino code  
✅ Interpret accelerometer and gyroscope data  
✅ Apply IMU data to detect mission events  
✅ Debug basic hardware issues  
✅ Design a simple sensor-based system  

---

## 🎉 Success Indicators

Students have mastered the material when they can:

1. Wire the sensor without referring to notes
2. Predict what data will look like before running code
3. Explain to a peer how the code works
4. Propose a new application for the sensor
5. Debug a wiring issue independently

---

**Good luck with your lessons! 🚀**

*Questions? Check README.md or experiment with the code!*
