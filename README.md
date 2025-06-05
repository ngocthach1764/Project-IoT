# 🚀 Project IoT: Smart Classroom

## 🔧 Platform 
- **Core IoT Platform with link: [app.coreiot.io](https://app.coreiot.io)**

---

## 📡 Hardware Components
- **Yolo UNO ESP32**
- **DHT20 Temperature and Humidity Sensor**
- **Analog Light Sensor**
- **Single LED**
- **RGB LED**
- **Cooling FAN**
- **I2C LCD Display**
- **Dual Push Button**
- **MQ-135 Air Quality Sensor**
- **MQ-2 Gas Sensor**

---

## 🎯 System Functions

1. **Environmental Measurement**
   - Collect temperature, humidity, and light intensity data from sensors.

2. **Remote Control of Devices via ThingsBoard**
   - Turn on/off single LED.
   - Turn on/off blinking LED as an alarm.
   - Turn on/off FAN.
   - Adjust the FAN speed when the FAN is turned on.

3. **Display Device Status on ThingsBoard**
   - Monitor the status of LED, blinking LED, FAN, and FAN speed.

4. **Automatic LED Control According to Ambient Light**
   - When it gets darker: the system automatically turns on the LED.
   - When it gets brighter: the system automatically turns off the LED.

5. **Automatic FAN Control According to Ambient Temperature**
   - When the temperature is higher than 30°C: the system automatically turns on the FAN.
   - When the temperature is lower than 25°C: the system automatically turns off the FAN.

6. **Manual Device Control Using Physical Button with Debounce Handling**
   - Turn on/off the LED using a physical push button with debounce logic.
   - Turn on/off the FAN using a physical push button with debounce logic.

7. **Air Quality Evaluation**
   - Measure the concentration of gases such as CO, CO₂, NH₄, Alcohol, Toluene, and Acetone to evaluate air quality.
   - Visual indication using RGB LED:
     - 🟢 Green: Good air quality
     - 🔴 Red: Poor air quality (any gas exceeds safe threshold)

8. **Gas Detection Alert**
   - Detect dangerous gases such as H₂, CO, LPG, Alcohol, and Propane.
   - Activate blinking LED alert when gas exceeds danger threshold.

9. **LCD Display**
   - The system automatically cycles every **4 seconds** between the following **4 main screens**:

     | 🖥️ Screen | 📋 Content                                                |
     |-----------|------------------------------------------------------------|
     | 1         | **School Name**, **Class Name**                            |
     | 2         | **Total Students**, **Number of Absent Students**          |
     | 3         | **Temperature (°C)**, **Humidity (%)**                     |
     | 4         | **Light Intensity (0–4950 ADC)**, **Fan Speed (0–255 PWM)** |

   - ⚠️ If any **gas level** (CO, CO₂, NH₄, Alcohol, Toluene, Acetone, H₂, LPG, or Propane) exceeds its **safe threshold**, the system **interrupts** the normal display to enter **alert mode**:
     - Displays a **Gas Warning Screen** for each detected hazardous gas, switching every **2 seconds**.
     - Continues warning cycle until **all gas levels return to safe thresholds**.

   - ✅ When all gas levels are safe again, the system **automatically resumes** displaying the **4 normal screens** described above.

10. **OTA Firmware Update**
   - If the update link is different from the previous one, the firmware is updated automatically.
   - If the same, the update is skipped.

11. **Wi-Fi Auto-Reconnection**
   - The system monitors Wi-Fi status.
   - Automatically reconnects when disconnected, without restarting the ESP32.

12. **ThingsBoard Auto-Reconnection**
   - Manages ThingsBoard MQTT connection.
   - Reconnects automatically if the connection is lost and resumes operation.

13. **RTOS-Based Task Management**
   - All features are implemented as **FreeRTOS tasks** for multitasking.

---

## ⚙️ Conflict Handling Mechanism Between Automatic and Manual Control

| **Ambient Light Condition** | **User RPC/Button Control** | **Device Status** | **System Mode** |
|-----------------------------|-----------------------------|-------------------|-----------------|
| Dark                        | No RPC/Button control       | LED ON            | Automatic       |
| Dark                        | RPC/Button turns off LED    | LED OFF           | Manual          |
| Bright                      | No RPC/Button control       | LED OFF           | Automatic       |
| Bright                      | RPC/Button turns on LED     | LED ON            | Manual          |

- When the user controls the device manually (via RPC or physical button), the system switches to **Manual Mode** and gives priority to the user.
- When environmental conditions change (e.g. bright after dark), the system will resume **Automatic Mode** if no manual control is detected.

> **FAN and blinking LED controls follow the same logic**, with FAN also supporting PWM (0–255) speed control.

---

