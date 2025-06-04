#define LED_PIN GPIO_NUM_1
#define FAN_PIN GPIO_NUM_2
#define LED_BLINK_PIN GPIO_NUM_48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define LIGHT_SENSOR_PIN GPIO_NUM_6
#define BUTTON_LED_PIN GPIO_NUM_8
#define BUTTON_FAN_PIN GPIO_NUM_9
#define LED_RGB_PIN GPIO_NUM_10
#define NUM_LED_RGB 4
#define MQ135_PIN GPIO_NUM_3
#define MQ2_PIN GPIO_NUM_4

#define FAN_CHANNEL 0
#define FAN_FREQ 25000 // 25kHz frequency for fan control
#define FAN_RESOLUTION 8 // 8-bit resolution for PWM

#define BOARD "Yolo UNO ESP32-S3"
#define VOLAYAGE_RESOLUTION 5 // 5V voltage resolution for ESP32
#define ADC_BIT_RESOLUTION 12 // 12-bit ADC resolution for ESP32
#define TYPE_MQ135 "MQ-135"
#define RATIO_MQ135 3.6 // Ratio of the sensor resistance in clean air
#define TYPE_MQ2 "MQ-2"
#define RATIO_MQ2 9.83  // / Ratio of the sensor resistance in clean air

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Button.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <MQUnifiedsensor.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>

constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";
constexpr char TOKEN[] = "RGt6uINezgdb0Rwra6Um";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char LED_STATE_ATTR[] = "ledState";
constexpr char FAN_STATE_ATTR[] = "fanState";
constexpr char LED_BLINK_ATTR[] = "ledBlink";
constexpr char FIRMWARE_URL_ATTR[] = "fw_url";
constexpr char FIRMWARE_VERSION_ATTR[] = "fw_version";
constexpr char FIRMWARE_TITLE_ATTR[] = "fw_title";

constexpr int LOW_LIGHT_THRESHOLD = 700; 
constexpr int HIGHT_LIGHT_THRESHOLD = 1300; 
constexpr int LOW_TEMP_THRESHOLD = 25; // Turn off fan
constexpr int HIGH_TEMP_THRESHOLD = 30; // Turn on fan

constexpr int H2_PPM_THRESHOLD = 10000;
constexpr int LPG_PPM_THRESHOLD = 300;  
constexpr int CO_PPM_THRESHOLD = 35;      
constexpr int ALCOHOL_PPM_THRESHOLD = 1000;
constexpr int PROPANE_PPM_THRESHOLD = 300;
constexpr int CO2_PPM_THRESHOLD = 1000;
constexpr int NH4_PPM_THRESHOLD = 25;
constexpr int ACETONE_PPM_THRESHOLD = 750;
constexpr int TOLUENE_PPM_THRESHOLD = 100;

constexpr uint8_t ButtonPins[] = { BUTTON_LED_PIN, BUTTON_FAN_PIN };

volatile bool ledState = false; 
volatile bool fanState = false;
volatile bool ledBlink = false;
volatile bool ledManualControl = false; 
volatile bool fanManualControl = false; 
volatile bool ledBlinkManualControl = false;
volatile int currentFanSpeed = 0; 
volatile bool fanSpeedControlEnabled = false;
volatile bool airQuality = true; // Assume air quality is good initially

String firmwareUrl = "";
String lastFirmwareUrl = ""; // Store the last firmware URL for OTA updates 

std::vector<String> exceededGases; // List of gases that exceed thresholds

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
LiquidCrystal_I2C lcd(0x21, 16, 2);
Adafruit_NeoPixel ledRGB(NUM_LED_RGB, LED_RGB_PIN, NEO_GRB + NEO_KHZ800);
MQUnifiedsensor MQ135(BOARD, VOLAYAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ135_PIN, TYPE_MQ135);
MQUnifiedsensor MQ2(BOARD, VOLAYAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ2_PIN, TYPE_MQ2);
Preferences preferences; // Preferences for storing firmware URL 

constexpr std::array<const char *, 6U> SHARED_ATTRIBUTES_LIST = {
    LED_STATE_ATTR,
    FAN_STATE_ATTR,
    LED_BLINK_ATTR,
    FIRMWARE_URL_ATTR,
    FIRMWARE_VERSION_ATTR,
    FIRMWARE_TITLE_ATTR
};

void setFanSpeed(int speed) {
    if (fanState) {
        speed = constrain(speed, 0, 255);
        currentFanSpeed = speed;
        ledcWrite(FAN_CHANNEL, speed);
    } else {
        currentFanSpeed = 0;
        ledcWrite(FAN_CHANNEL, 0);
    }
}

void setLedRGBColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LED_RGB; i++) {
        ledRGB.setPixelColor(i, ledRGB.Color(r, g, b));
    }
    ledRGB.show();
}

void updateLedRGB(bool airQuality) {
    if (airQuality) setLedRGBColor(0, 255, 0);
    else setLedRGBColor(255, 0, 0); 
}

void setupGasMQ135(float a, float b) {
    MQ135.setRegressionMethod(1); //_PPM = a * ratio^b
    MQ135.setA(a); 
    MQ135.setB(b);
}

void setupGasMQ2(float a, float b) {
    MQ2.setRegressionMethod(1); 
    MQ2.setA(a);
    MQ2.setB(b);
}

void calibrateSensorMQ135() {
    Serial.print("Calibrating MQ135...");
    float calcR0 = 0;
    for (int i = 0; i < 10; i++) {
        MQ135.update();
        calcR0 += MQ135.calibrate(RATIO_MQ135);
        Serial.print(".");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    MQ135.setR0(calcR0 / 10);
    Serial.println(" done!");

    if (isinf(calcR0)) {
        Serial.println("Warning: R0 is infinite! Check wiring.");
        while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    if (calcR0 == 0) {
        Serial.println("Warning: R0 is zero! Check wiring.");
        while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }

    MQ135.serialDebug(true);
}

void calibrateSensorMQ2() {
    Serial.print("Calibrating MQ2...");
    float calcR0 = 0;
    for (int i = 0; i < 10; i++) {
        MQ2.update();
        calcR0 += MQ2.calibrate(RATIO_MQ2);
        Serial.print(".");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    MQ2.setR0(calcR0 / 10);
    Serial.println(" done!");

    if (isinf(calcR0)) {
        Serial.println("Warning: R0 is infinite! Check wiring.");
        while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    if (calcR0 == 0) {
        Serial.println("Warning: R0 is zero! Check wiring.");
        while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }

    MQ2.serialDebug(true);
}

RPC_Response setLedSwitchState(const RPC_Data &data) {
    Serial.println("Received Switch state");
    ledState = data;
    Serial.print("Switch state change: ");
    Serial.println(ledState);
    
    int lightValue = analogRead(LIGHT_SENSOR_PIN);

    //RPC and sensor both want to turn on/off the light, so control is returned to the sensor.
    if ((ledState && lightValue < LOW_LIGHT_THRESHOLD) || 
        (!ledState && lightValue > HIGHT_LIGHT_THRESHOLD) ||
        (ledManualControl && lightValue >= LOW_LIGHT_THRESHOLD && lightValue <= HIGHT_LIGHT_THRESHOLD)) {
        ledManualControl = false; 
    } else ledManualControl = true;

    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    tb.sendAttributeData(LED_STATE_ATTR, ledState ? "ON" : "OFF");
    return RPC_Response("setLedSwitchValue", ledState);
}

RPC_Response setFanSwitchState(const RPC_Data &data) {
    Serial.println("Received Switch state");
    fanState = data;
    Serial.print("Switch state change: ");
    Serial.println(fanState);

    dht20.read();
    float temperature = dht20.getTemperature();

    // RPC and sensor both want to turn on/off the fan, so control is returned to the sensor.
    if ((fanState && temperature > HIGH_TEMP_THRESHOLD) ||
        (!fanState && temperature < LOW_TEMP_THRESHOLD) ||
        (fanManualControl && temperature >= LOW_TEMP_THRESHOLD && temperature <= HIGH_TEMP_THRESHOLD)) {
        fanManualControl = false;
    } else fanManualControl = true;

    if (currentFanSpeed > 0) setFanSpeed(fanState ? currentFanSpeed : 0);
    else setFanSpeed(fanState ? 255 : 0);
    
    tb.sendAttributeData(FAN_STATE_ATTR, fanState ? "ON" : "OFF");
    return RPC_Response("setFanSwitchValue", fanState);
}

RPC_Response setFanSpeedValue(const RPC_Data &data) {
    int speed = data;
    if (fanState) {
        setFanSpeed(speed);
        fanSpeedControlEnabled = true;
    }
    return RPC_Response("setFanSpeedValue", speed);
}

RPC_Response setLedBlinkState(const RPC_Data &data) {
    Serial.println("Received LED blink state");
    ledBlink = data;
    Serial.print("LED blink state change: ");
    Serial.println(ledBlink);

    // read H2 from MQ2 sensor
    setupGasMQ2(987.99, -2.162);
    MQ2.update();
    float ppmH2 = MQ2.readSensor();

    // read LPG from MQ2 sensor
    setupGasMQ2(574.25, -2.222);
    MQ2.update();
    float ppmLPG = MQ2.readSensor();

    // read CO from MQ2 sensor
    setupGasMQ2(36974, -3.109);
    MQ2.update();
    float ppmCO_MQ2 = MQ2.readSensor();

    // read Alcohol from MQ2 sensor
    setupGasMQ2(3616.1, -2.675);
    MQ2.update();
    float ppmAlcohol_MQ2 = MQ2.readSensor();

    // read Propane from MQ2 sensor
    setupGasMQ2(658.71, -2.168);
    MQ2.update();
    float ppmPropane = MQ2.readSensor();

    bool gasDetection = (ppmH2 > H2_PPM_THRESHOLD || ppmLPG > LPG_PPM_THRESHOLD ||
                            ppmCO_MQ2 > CO_PPM_THRESHOLD || ppmAlcohol_MQ2 > ALCOHOL_PPM_THRESHOLD ||
                            ppmPropane > PROPANE_PPM_THRESHOLD);
    
    // RPC and sensor both want to turn on/off the LED blink, so control is returned to the sensor.
    if ((ledBlink && gasDetection)|| (!ledBlink && !gasDetection)) {
        ledBlinkManualControl = false; 
    } else ledBlinkManualControl = true; 
    
    tb.sendTelemetryData("gasDetection", ledBlink ? "Danger" : "Safe");
    tb.sendAttributeData(LED_BLINK_ATTR, ledBlink ? "ON" : "OFF");
    return RPC_Response("setLedBlinkValue", ledBlink);
}

const std::array<RPC_Callback, 4U> callbacks = {
    RPC_Callback{ "setLedSwitchValue", setLedSwitchState },
    RPC_Callback{ "setFanSwitchValue", setFanSwitchState },
    RPC_Callback{ "setLedBlinkValue", setLedBlinkState },
    RPC_Callback{ "setFanSpeed", setFanSpeedValue }
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
        if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
            ledState = it->value().as<bool>();
            digitalWrite(LED_PIN, ledState);
            Serial.print("LED state is set to: ");
            Serial.println(ledState);
        } else if (strcmp(it->key().c_str(), FAN_STATE_ATTR) == 0) {
            fanState = it->value().as<bool>();
            digitalWrite(FAN_PIN, fanState);
            Serial.print("Fan state is set to: ");
            Serial.println(fanState);
        } else if (strcmp(it->key().c_str(), LED_BLINK_ATTR) == 0) {
            ledBlink = it->value().as<bool>();
            Serial.print("LED blink state is set to: ");
            Serial.println(ledBlink);
        } else if (strcmp(it->key().c_str(), FIRMWARE_URL_ATTR) == 0) {
            firmwareUrl = it->value().as<String>();
            Serial.print("Firmware URL is set to: ");
            Serial.println(firmwareUrl);
        } else if (strcmp(it->key().c_str(), FIRMWARE_VERSION_ATTR) == 0) {
            String firmwareVersion = it->value().as<String>();
            Serial.print("Firmware version is set to: ");
            Serial.println(firmwareVersion);
        } else if (strcmp(it->key().c_str(), FIRMWARE_TITLE_ATTR) == 0) {
            String firmwareTitle = it->value().as<String>();
            Serial.print("Firmware title is set to: ");
            Serial.println(firmwareTitle);
        }
    }
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void initWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
    }
    Serial.println("WiFi Connected!");
}

void taskWiFiConnect(void *pvParameters) {
    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Reconnecting to WiFi...");
            WiFi.disconnect();
            initWiFi();

            // Send WiFi connection details to ThingsBoard
            tb.sendAttributeData("rssi", WiFi.RSSI());
            tb.sendAttributeData("channel", WiFi.channel());
            tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
            tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
            tb.sendAttributeData("ssid", WiFi.SSID().c_str());
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void taskCoreIOTConnect(void *pvParameters) {
    while (1) {
        if (!tb.connected()) {
            Serial.println("Connecting to CoreIOT...");
            if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
                Serial.println("Connected to CoreIOT!");
                tb.sendAttributeData("macAdress", WiFi.macAddress().c_str());

                Serial.println("Subscribing for RPC calls...");
                if (tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
                    Serial.println("RPC Subscription successful!");
                } else {
                    Serial.println("Failed to subscribe for RPC calls!");
                }

                Serial.println("Subscribing for shared attributes...");
                if (tb.Shared_Attributes_Subscribe(attributes_callback)) {
                    Serial.println("Shared Attributes Subscription successful!");
                } else {
                    Serial.println("Failed to subscribe for shared attributes!");
                }

                Serial.println("Requesting shared attributes...");
                if (tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
                    Serial.println("Shared Attributes Request successful!");
                } else {
                    Serial.println("Failed to request shared attributes!");
                }
            } else {
                Serial.println("Failed to connect to CoreIOT");
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void taskBlinkLed(void *pvParameters) {
    while (1) {
        if (ledBlink) {
            digitalWrite(LED_BLINK_PIN, HIGH);
            for (int i = 0; i < 50; i++) {
                if (!ledBlink) break;
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            digitalWrite(LED_BLINK_PIN, LOW);
            for (int i = 0; i < 50; i++) {
                if (!ledBlink) break;
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        } else {
            digitalWrite(LED_BLINK_PIN, LOW);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void taskSendTelemetry(void *pvParameters) {
    while (1) {
        // Read temperature and humidity from DHT20 sensor
        dht20.read();
        float temperature = dht20.getTemperature();
        float humidity = dht20.getHumidity();

        if (!isnan(temperature) && !isnan(humidity)) {
            Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
            tb.sendTelemetryData("temperature", temperature);
            tb.sendTelemetryData("humidity", humidity);

            updateLedRGB(temperature); // Update LED RGB color based on temperature
        } else {
            Serial.println("Failed to read from DHT sensor!");
        }

        // Read light sensor value
        int lightValue = analogRead(LIGHT_SENSOR_PIN);
        if (lightValue < 0 || lightValue > 4095) {
            Serial.println("Invalid light sensor value!");
        } else {
            Serial.printf("Light Sensor Value: %d\n", lightValue);
            tb.sendTelemetryData("lightSensor", lightValue);
        }

        // Send fan speed telemetry
        if (fanState) {
            tb.sendTelemetryData("fanSpeed", currentFanSpeed);
            Serial.printf("Fan Speed: %d\n", currentFanSpeed);
        } else {
            tb.sendTelemetryData("fanSpeed", 0);
            Serial.println("Fan is OFF, speed set to 0");
        }

        // read CO from MQ135 sensor
        setupGasMQ135(605.18, -3.937);
        MQ135.update();
        float ppmCO_MQ135 = MQ135.readSensor();
        Serial.printf("CO PPM from MQ135 sensor: %.2f\n", ppmCO_MQ135);
        tb.sendTelemetryData("ppmCO_MQ135", ppmCO_MQ135);

        // read CO2 from MQ135 sensor
        setupGasMQ135(110.47, -2.862);
        MQ135.update();
        float ppmCO2 = MQ135.readSensor();
        Serial.printf("CO2 PPM: %.2f\n", ppmCO2);      
        tb.sendTelemetryData("ppmCO2", ppmCO2);

        // read NH4 from MQ135 sensor
        setupGasMQ135(102.2, -2.473);
        MQ135.update();
        float ppmNH4 = MQ135.readSensor();
        Serial.printf("NH4 PPM: %.2f\n", ppmNH4);
        tb.sendTelemetryData("ppmNH4", ppmNH4);

        // read Alcohol from MQ135 sensor
        setupGasMQ135(77.255, -3.18);
        MQ135.update();         
        float ppmAlcohol_MQ135 = MQ135.readSensor();
        Serial.printf("Alcohol PPM from MQ135 sensor: %.2f\n", ppmAlcohol_MQ135);
        tb.sendTelemetryData("ppmAlcohol_MQ135", ppmAlcohol_MQ135);

        // read Toluene from MQ135 sensor
        setupGasMQ135(44.947, -3.445);
        MQ135.update();
        float ppmToluene = MQ135.readSensor();
        Serial.printf("Toluene PPM: %.2f\n", ppmToluene);
        tb.sendTelemetryData("ppmToluene", ppmToluene);

        // read Acetone from MQ135 sensor
        setupGasMQ135(34.668, -3.369);
        MQ135.update();
        float ppmAcetone = MQ135.readSensor();
        Serial.printf("Acetone PPM: %.2f\n", ppmAcetone);
        tb.sendTelemetryData("ppmAcetone", ppmAcetone);

        // Check air quality 
        airQuality = (ppmCO_MQ135 < CO_PPM_THRESHOLD && ppmCO2 < CO2_PPM_THRESHOLD &&
                      ppmNH4 < NH4_PPM_THRESHOLD && ppmAlcohol_MQ135 < ALCOHOL_PPM_THRESHOLD &&
                      ppmToluene < TOLUENE_PPM_THRESHOLD && ppmAcetone < ACETONE_PPM_THRESHOLD);
        Serial.printf("Air Quality: %s\n", airQuality ? "Good" : "Bad");
        tb.sendTelemetryData("airQuality", airQuality ? "Good" : "Bad");

        updateLedRGB(airQuality);

        // read H2 from MQ2 sensor
        setupGasMQ2(987.99, -2.162);
        MQ2.update();
        float ppmH2 = MQ2.readSensor();
        Serial.printf("H2 PPM: %.2f\n", ppmH2);
        tb.sendTelemetryData("ppmH2", ppmH2);

        // read LPG from MQ2 sensor
        setupGasMQ2(574.25, -2.222);
        MQ2.update();
        float ppmLPG = MQ2.readSensor();
        Serial.printf("LPG PPM: %.2f\n", ppmLPG);   
        tb.sendTelemetryData("ppmLPG", ppmLPG);

        // read CO from MQ2 sensor
        setupGasMQ2(36974, -3.109);
        MQ2.update();
        float ppmCO_MQ2 = MQ2.readSensor();
        Serial.printf("CO PPM from MQ2 sensor: %.2f\n", ppmCO_MQ2);
        tb.sendTelemetryData("ppmCO_MQ2", ppmCO_MQ2);

        // read Alcohol from MQ2 sensor
        setupGasMQ2(3616.1, -2.675);
        MQ2.update();
        float ppmAlcohol_MQ2 = MQ2.readSensor();
        Serial.printf("Alcohol PPM from MQ2 sensor: %.2f\n", ppmAlcohol_MQ2);
        tb.sendTelemetryData("ppmAlcohol_MQ2", ppmAlcohol_MQ2);

        // read Propane from MQ2 sensor
        setupGasMQ2(658.71, -2.168);
        MQ2.update();
        float ppmPropane = MQ2.readSensor();
        Serial.printf("Propane PPM: %.2f\n", ppmPropane);
        tb.sendTelemetryData("ppmPropane", ppmPropane);

        // Check gas detection
        bool gasDetection = (ppmH2 > H2_PPM_THRESHOLD || ppmLPG > LPG_PPM_THRESHOLD ||
                             ppmCO_MQ2 > CO_PPM_THRESHOLD || ppmAlcohol_MQ2 > ALCOHOL_PPM_THRESHOLD ||
                             ppmPropane > PROPANE_PPM_THRESHOLD);
        
        if ((ledBlink && gasDetection)|| (!ledBlink && !gasDetection)) {
            ledBlinkManualControl = false; 
        } 

        if (!ledBlinkManualControl) {
            if (gasDetection) {
                ledBlink = true;
                Serial.println("LED blink ON due to gas detection");
            } else {
                ledBlink = false;
                Serial.println("LED blink OFF, no gas detected");
            }
            tb.sendTelemetryData("gasDetection", gasDetection ? "Danger" : "Safe");
            tb.sendAttributeData(LED_BLINK_ATTR, ledBlink ? "ON" : "OFF");
        }

        // Add gas to list if exceeded thresholds
        exceededGases.clear();
        if (ppmH2 > H2_PPM_THRESHOLD) exceededGases.push_back("H2");
        if (ppmLPG > LPG_PPM_THRESHOLD) exceededGases.push_back("LPG");
        if (ppmCO_MQ2 > CO_PPM_THRESHOLD || ppmCO_MQ135 > CO_PPM_THRESHOLD) exceededGases.push_back("CO");
        if (ppmAlcohol_MQ2 > ALCOHOL_PPM_THRESHOLD || ppmAlcohol_MQ135 > ALCOHOL_PPM_THRESHOLD) exceededGases.push_back("Alcohol");
        if (ppmPropane > PROPANE_PPM_THRESHOLD) exceededGases.push_back("Propane");
        if (ppmCO2 > CO2_PPM_THRESHOLD) exceededGases.push_back("CO2");
        if (ppmNH4 > NH4_PPM_THRESHOLD) exceededGases.push_back("NH4");
        if (ppmAcetone > ACETONE_PPM_THRESHOLD) exceededGases.push_back("Acetone");
        if (ppmToluene > TOLUENE_PPM_THRESHOLD) exceededGases.push_back("Toluene");

        // Send location telemetry data
        double latitude = 10.880018410410052;
        double longitude = 106.80633605864662;
        tb.sendTelemetryData("latitude", latitude);
        tb.sendTelemetryData("longitude", longitude);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void taskAutoLedControl(void *pvParameters) {
    while (1) {
        int lightValue = analogRead(LIGHT_SENSOR_PIN);

        if ((ledState && lightValue < LOW_LIGHT_THRESHOLD) || 
            (!ledState && lightValue > HIGHT_LIGHT_THRESHOLD)) {
            ledManualControl = false; 
        }

        if (!ledManualControl) {
            if (lightValue < LOW_LIGHT_THRESHOLD) {
                ledState = true;
                Serial.println("LED turned ON due to low light");
            } else if (lightValue > HIGHT_LIGHT_THRESHOLD) {
                ledState = false;
                Serial.println("LED turned OFF due to sufficient light");
            }
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            tb.sendAttributeData(LED_STATE_ATTR, ledState ? "ON" : "OFF");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void taskAutoFanControl(void *pvParameters) {
    while (1) {
        dht20.read();
        float temperature = dht20.getTemperature();
        
        if (!isnan(temperature)) {
            if ((fanState && temperature > HIGH_TEMP_THRESHOLD) || 
                (!fanState && temperature < LOW_TEMP_THRESHOLD) ||
                (fanManualControl && temperature >= LOW_TEMP_THRESHOLD && temperature <= HIGH_TEMP_THRESHOLD)) {
                fanManualControl = false; 
            }

            if (!fanState) fanSpeedControlEnabled = false;

            if (!fanManualControl) {
                if (temperature > HIGH_TEMP_THRESHOLD) {
                    fanState = true;
                    Serial.println("Fan turned ON due to high temperature");
                    if (fanSpeedControlEnabled) {
                        setFanSpeed(currentFanSpeed);
                    } else {
                        setFanSpeed(255); 
                    }
                } else if (temperature < LOW_TEMP_THRESHOLD) {
                    fanState = false; 
                    Serial.println("Fan turned OFF due to low temperature");
                    fanSpeedControlEnabled = false;
                    setFanSpeed(0);
                }

                tb.sendAttributeData(FAN_STATE_ATTR, fanState ? "ON" : "OFF");
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void taskReadButton(void *pvParameters) {
    while (1) {
        getKeyInput();

        if (isButtonPressed(0)) {
            Serial.println("Button LED pressed!");
            ledState = !ledState;
            ledManualControl = true;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            tb.sendAttributeData(LED_STATE_ATTR, ledState ? "ON" : "OFF");
        }

        if (isButtonPressed(1)) {
            Serial.println("Button FAN pressed!");
            fanState = !fanState;
            fanManualControl = true;
            setFanSpeed(fanState ? 255 : 0);
            tb.sendAttributeData(FAN_STATE_ATTR, fanState ? "ON" : "OFF");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void taskLCDDisplay(void *pvParameters) {
    int screen = 0;
    const char* schoolName = "HCMUT";
    const char* classroomName = "H6-105";
    int totalStudents = 32;
    int absentStudents = 0;

    unsigned long lastGasDisplayTime = 0;
    int gasDisplayIndex = 0;

    while (1) {
        lcd.clear();
        if (ledBlink && !exceededGases.empty()) {
            lcd.setCursor(2, 0);
            lcd.print("!! WARNING !!");

            String gasName = exceededGases[gasDisplayIndex];
            int padding = (16 - gasName.length()) / 2;
            padding = max(padding, 0);
            lcd.setCursor(padding, 1);
            lcd.print(gasName);

            if (millis() - lastGasDisplayTime >= 2000) {
                gasDisplayIndex = (gasDisplayIndex + 1) % exceededGases.size();
                lastGasDisplayTime = millis();
            }
        } else {
            switch (screen) {
                case 0:
                    lcd.setCursor(0, 0);
                    lcd.print("School: ");
                    lcd.print(schoolName);
                    lcd.setCursor(0, 1);
                    lcd.print("Class: ");
                    lcd.print(classroomName);
                    break;
                case 1:
                    lcd.setCursor(0, 0);
                    lcd.print("Students: ");
                    lcd.print(totalStudents);
                    lcd.setCursor(0, 1);       
                    lcd.print("Absent: ");
                    lcd.print(absentStudents);
                    break;
                case 2: {
                    dht20.read();  
                    float temperature = dht20.getTemperature();
                    float humidity = dht20.getHumidity();

                    lcd.setCursor(0, 0);
                    lcd.print("Temp: ");
                    lcd.print(temperature, 2);
                    lcd.print(" C");
                    lcd.setCursor(0, 1);
                    lcd.print("Hum: ");
                    lcd.print(humidity, 2);
                    lcd.print(" % ");
                    break;
                }
                case 3: {
                    lcd.setCursor(0, 0);
                    lcd.print("Light: ");
                    int lightValue = analogRead(LIGHT_SENSOR_PIN);
                    lcd.print(lightValue);
                    lcd.print(" ADC");
                    lcd.setCursor(0, 1);
                    lcd.print("Fan Speed: ");
                    lcd.print(currentFanSpeed);
                    break;
                }
            }
            screen = (screen + 1) % 4;
            vTaskDelay(4000 / portTICK_PERIOD_MS); 
            continue;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void taskOTAUpdate(void *pvParameters) {
    while (1) {
        if (firmwareUrl != "" && firmwareUrl != lastFirmwareUrl) {
            Serial.println("Starting OTA Update from URL: " + firmwareUrl);

            HTTPClient http;
            http.begin(firmwareUrl);
            int httpCode = http.GET();

            if (httpCode == HTTP_CODE_OK) {
                int contentLength = http.getSize();
                WiFiClient *stream = http.getStreamPtr();

                if (contentLength > 0) {
                    bool canBegin = Update.begin(contentLength);
                    if (canBegin) {
                        size_t written = Update.writeStream(*stream);
                        if (written == contentLength) {
                            Serial.println("Firmware written successfully.");
                        } else {
                            Serial.printf("Written only %d/%d bytes.\n", (int)written, contentLength);
                        }

                        if (Update.end()) {
                            if (Update.isFinished()) {
                                Serial.println("OTA Update successful. Rebooting...");
                                preferences.begin("ota", false);
                                preferences.putString("last_url", firmwareUrl);
                                preferences.end();
                                lastFirmwareUrl = firmwareUrl;
                                http.end();
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                                ESP.restart();
                            } else {
                                Serial.println("OTA Update not finished properly.");
                            }
                        } else {
                            Serial.printf("Update failed: %s\n", Update.errorString());
                        }
                    } else {
                        Serial.println("Not enough space to begin OTA");
                    }
                } else {
                    Serial.println("Content length is not correct");
                }
            } else {
                Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
            }
            http.end();
        } else {
            Serial.println("Firmware URLs are the same. Skipping OTA update.");
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS); 
    }
}


void taskThingsBoardLoop(void *pvParameters) {
    while (1) {
        tb.loop();
        mqttClient.loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);
    initWiFi();
    pinMode(LED_PIN, OUTPUT);

    pinMode(FAN_PIN, OUTPUT);
    ledcSetup(FAN_CHANNEL, FAN_FREQ, FAN_RESOLUTION);
    ledcAttachPin(FAN_PIN, FAN_CHANNEL);

    pinMode(LED_BLINK_PIN, OUTPUT);
    Wire.begin(SDA_PIN, SCL_PIN);
    dht20.begin();

    buttonInit(ButtonPins, sizeof(ButtonPins) / sizeof(ButtonPins[0]));

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LCD Ready!");

    ledRGB.begin();
    ledRGB.show();

    MQ135.init();
    calibrateSensorMQ135();

    MQ2.init();
    calibrateSensorMQ2();

    preferences.begin("ota", true); // true for read-only mode
    lastFirmwareUrl = preferences.getString("last_url", "");
    preferences.end();


    xTaskCreate(taskWiFiConnect, "WiFiConnect", 4096, NULL, 1, NULL);
    xTaskCreate(taskCoreIOTConnect, "CoreIOTConnect", 4096, NULL, 1, NULL);
    xTaskCreate(taskBlinkLed, "BlinkLed", 4096, NULL, 1, NULL);
    xTaskCreate(taskSendTelemetry, "SendTelemetry", 4096, NULL, 1, NULL);
    xTaskCreate(taskAutoLedControl, "AutoLedControl", 4096, NULL, 1, NULL);
    xTaskCreate(taskAutoFanControl, "AutoFanControl", 4096, NULL, 1, NULL);
    xTaskCreate(taskReadButton, "ReadButton", 4096, NULL, 1, NULL);
    xTaskCreate(taskLCDDisplay, "LCDDisplay", 4096, NULL, 1, NULL);
    xTaskCreate(taskOTAUpdate, "OTAUpdate", 8192, NULL, 1, NULL);
    xTaskCreate(taskThingsBoardLoop, "ThingsBoardLoop", 4096, NULL, 1, NULL);
}

void loop() {
    
}