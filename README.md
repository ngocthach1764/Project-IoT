# 🚀 Project IoT: Smart Classroom

## 🔧 Platform 
- **Core IoT Platform with link: app.coreiot.io**
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
   - Collect temperature, humidity, and LED intensity data from sensors.

2. **Remote Control of Devices via ThingsBoard**
   - Turn on/off single LED.
   - Turn on/off blinking LED as an alarm.
   - Turn on/off FAN.
   - Adjust the FAN speed when the FAN is turned on.

3. **Display Device Status on ThingsBoard**
   - Monitor the status of LED, blinking LED, FAN and monitor speed FAN.

4. **Automatic LED Control According to Ambient LED**
   - When it gets darker: the system automatically turns on the LED.
   - When it gets brighter: the system automatically turns off the LED.

5.  **Automatic FAN Control According to Ambient Temperature**  
   - When the temperature is higher than 30°C: the system automatically turns on the FAN.  
   - When the temperature is lower than 25°C: the system automatically turns off the FAN.

6. **Manual Device Control Using Physical Button with Debounce Handling**
   - Turn on/off the LED using a physical push button with debounce logic.
   - Turn on/off the FAN using a physical push button with debounce logic.

7. **Air Quality Evaluation**
   - Measure the concentration of gases such as CO, CO₂, NH₄, Alcohol, Toluene, and Acetone to evaluate air quality.
   - Visual indication using RGB LED with Green = Good air quality and Red = Poor air quality (any gas exceeds safe threshold).

8. **Gas Detection Alert**
   - Measure the concentration of gases such as H₂, CO, LPG, Alcohol, and Propane to detect potential fire hazards.
   - Visual alert using blinking LED is activated when any gas exceeds danger threshold.

9. **LCD Display**
   - The system displays information on an LCD screen that automatically switches between multiple screens every 4 seconds:

| 🖥️ Screen Number | 📋 Display Content                            |
|-------------------|-----------------------------------------------|
| 1                 | **School Name**, **Class Name**                |
| 2                 | **Total Students**, **Number of Absent Students** |
| 3                 | **Temperature (°C)**, **Humidity (%)**         |
| 4                 | **Light Intensity (0-4950 ADC)**, **Fan Speed (0-255 PWM)**  |

   - If **gas levels (CO, CO₂, NH₄, Alcohol, Toluene, Acetone, H₂, LPG, or Propane) exceed the safe threshold**, the system will immediately display a gas warning screen every 2 seconds, rotating through each detected hazardous gas. Once all gas levels return to safe thresholds, the system resumes displaying the regular main screens as described above.

10. **OTA firmware update**
   - If different from the previous link, update it.
   - If the same as the previous link, skip it.
---

## ⚙️ Conflict Handling Mechanism between Automatic and Manual Control

The system operates according to the table below:

| **Ambient LED Condition**        | **User RPC/Button Control**          | **Device Status**     | **System Mode**    |
|-----------------------------------|-------------------------------|----------------------|--------------------|
| Dark (automatically turn on LED) | No RPC/Button control               | LED ON             | Automatic          |
| Dark (automatically turn on LED) | RPC/Button turns off LED           | LED OFF            | Manual             |
| Bright (automatically turn off LED) | No RPC/Button control             | LED OFF            | Automatic          |
| Bright (automatically turn off LED) | RPC/Button turns on LED          | LED ON             | Manual             |

- When the user intervenes by RPC/Button (turning the LED on/off), the system switches to **manual mode** and prioritizes the user control.
- When the system returns to the opposite environmental condition (for example, bright after dark), the system automatically controls again if there is no user intervention.

**LedBlink control and FAN control works similarly, with the FAN having additional support for PWM (0-255) speed control.**

---