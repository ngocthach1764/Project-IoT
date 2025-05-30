# 🚀 Project IoT: Smart Classroom

## 📡 Hardware Components
- **Yolo UNO ESP32**
- **DHT20 Temperature and Humidity Sensor**
- **Analog Light Sensor**
- **Single LED**
- **RGB LED**
- **Fan**
- **LCD Display**

---

## 🎯 System Functions

1. **Environmental Measurement**
   - Collect temperature, humidity, and light intensity data from sensors.

2. **Remote Control of Devices via ThingsBoard**
   - Turn on/off single LED.
   - Turn on/off blinking LED as an alarm.
   - Turn on/off fan.

3. **Display Device Status on ThingsBoard**
   - Monitor the status of LED, blinking LED, and fan.

4. **Automatic Light Control According to Ambient Light**
   - When it gets darker: the system automatically turns on the light.
   - When it gets brighter: the system automatically turns off the light.

---

## ⚙️ Conflict Handling Mechanism between Automatic and Manual Control

The system operates according to the table below:

| **Ambient Light Condition**        | **User RPC Control**          | **Device Status**     | **System Mode**    |
|-----------------------------------|-------------------------------|----------------------|--------------------|
| Dark (automatically turn on light) | No RPC control                | Light ON             | Automatic          |
| Dark (automatically turn on light) | RPC turns off light           | Light OFF            | Manual             |
| Bright (automatically turn off light) | No RPC control             | Light OFF            | Automatic          |
| Bright (automatically turn off light) | RPC turns on light          | Light ON             | Manual             |

- When the user intervenes by RPC (turning the light on/off), the system switches to **manual mode** and prioritizes the user control.
- When the system returns to the opposite environmental condition (for example, bright after dark), the system automatically controls again if there is no user intervention.

**Fan control works similarly, but with an added feature to control speed via PWM (0-255):**

- In **automatic mode**, the fan turns on/off based on temperature and runs at a preset default speed.
- In **manual mode**, the user has full control including adjusting the **PWM speed (0-255)**.

---