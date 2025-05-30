#define LED_PIN GPIO_NUM_1
#define FAN_PIN GPIO_NUM_2
#define LED_BLINK_PIN GPIO_NUM_48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define LIGHT_SENSOR_PIN GPIO_NUM_6

#define FAN_CHANNEL 0
#define FAN_FREQ 25000 // 25kHz frequency for fan control
#define FAN_RESOLUTION 8 // 8-bit resolution for PWM

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

constexpr char WIFI_SSID[] = "Pnt";
constexpr char WIFI_PASSWORD[] = "123456789";
constexpr char TOKEN[] = "RGt6uINezgdb0Rwra6Um";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char LED_STATE_ATTR[] = "ledState";
constexpr char FAN_STATE_ATTR[] = "fanState";
constexpr char LED_BLINK_ATTR[] = "ledBlink";

constexpr int LOW_LIGHT_THRESHOLD = 700; 
constexpr int HIGHT_LIGHT_THRESHOLD = 1300; 
constexpr int LOW_TEMP_THRESHOLD = 25; // Turn off fan
constexpr int HIGH_TEMP_THRESHOLD = 30; // Turn on fan

volatile bool ledState = false; 
volatile bool fanState = false;
volatile bool ledBlink = false;
volatile bool ledManualControl = false; 
volatile bool fanManualControl = false; 

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

constexpr std::array<const char *, 4U> SHARED_ATTRIBUTES_LIST = {
    LED_STATE_ATTR,
    FAN_STATE_ATTR,
    LED_BLINK_ATTR
};

void setFanSpeed(int speed) {
    if (fanState) {
        speed = constrain(speed, 0, 255);
        ledcWrite(FAN_CHANNEL, speed);
        Serial.printf("Fan speed set to: %d\n", speed);
    } else {
        ledcWrite(FAN_CHANNEL, 0);
        Serial.println("Fan is OFF, speed set to 0");
    }
}

RPC_Response setLedSwitchState(const RPC_Data &data) {
    Serial.println("Received Switch state");
    ledState = data;
    Serial.print("Switch state change: ");
    Serial.println(ledState);
    
    int lightValue = analogRead(LIGHT_SENSOR_PIN);

    //RPC and sensor both want to turn on/off the light, so control is returned to the sensor.
    if ((ledState && lightValue < LOW_LIGHT_THRESHOLD) || 
        (!ledState && lightValue > HIGHT_LIGHT_THRESHOLD)) {
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

    if ((fanState && temperature > HIGH_TEMP_THRESHOLD) ||
        (!fanState && temperature < LOW_TEMP_THRESHOLD)) {
        fanManualControl = false;
    } else {
        fanManualControl = true;
    }

    // If fan is turned on, set speed to maximum (255). If fan is turned off, set speed to 0
    setFanSpeed(fanState ? 255 : 0);
    tb.sendAttributeData(FAN_STATE_ATTR, fanState ? "ON" : "OFF");
    return RPC_Response("setFanSwitchValue", fanState);
}

RPC_Response setFanSpeedValue(const RPC_Data &data) {
    if (!fanManualControl) {
        Serial.println("Fan is in automatic mode, cannot set speed via RPC.");
        return RPC_Response("error", "Fan is in automatic mode, cannot set speed.");
    }

    int speed = data;
    setFanSpeed(speed);
    return RPC_Response("setFanSpeedValue", speed);
}

RPC_Response setLedBlinkState(const RPC_Data &data) {
    Serial.println("Received LED blink state");
    ledBlink = data;
    Serial.print("LED blink state change: ");
    Serial.println(ledBlink);
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
        }
    }
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
    }
    Serial.println("WiFi Connected!");
}

void TaskWiFiConnect(void *pvParameters) {
    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Reconnecting to WiFi...");
            WiFi.disconnect();
            InitWiFi();
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void TaskCoreIOTConnect(void *pvParameters) {
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

void TaskBlinkLed(void *pvParameters) {
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

void TaskSendTelemetry(void *pvParameters) {
    while (1) {
        // Read temperature and humidity from DHT20 sensor
        dht20.read();
        float temperature = dht20.getTemperature();
        float humidity = dht20.getHumidity();

        if (!isnan(temperature) && !isnan(humidity)) {
            Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
            tb.sendTelemetryData("temperature", temperature);
            tb.sendTelemetryData("humidity", humidity);
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

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void TaskAutoLedControl(void *pvParameters) {
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

void TaskAutoFanControl(void *pvParameters) {
    while (1) {
        dht20.read();
        float temperature = dht20.getTemperature();
        
        if (!isnan(temperature)) {
            if ((fanState && temperature > HIGH_TEMP_THRESHOLD) || 
                (!fanState && temperature < LOW_TEMP_THRESHOLD)) {
                fanManualControl = false; 
            }

            if (!fanManualControl) {
                if (temperature > HIGH_TEMP_THRESHOLD) {
                    fanState = true;
                    Serial.println("Fan turned ON due to high temperature");
                } else if (temperature < LOW_TEMP_THRESHOLD) {
                    fanState = false;
                    Serial.println("Fan turned OFF due to low temperature");
                }
                setFanSpeed(fanState ? 255 : 0);
                tb.sendAttributeData(FAN_STATE_ATTR, fanState ? "ON" : "OFF");
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void TaskThingsBoardLoop(void *pvParameters) {
    while (1) {
        tb.loop();
        mqttClient.loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);
    InitWiFi();
    pinMode(LED_PIN, OUTPUT);

    pinMode(FAN_PIN, OUTPUT);
    ledcSetup(FAN_CHANNEL, FAN_FREQ, FAN_RESOLUTION);
    ledcAttachPin(FAN_PIN, FAN_CHANNEL);

    pinMode(LED_BLINK_PIN, OUTPUT);
    Wire.begin(SDA_PIN, SCL_PIN);
    dht20.begin();

    xTaskCreate(TaskWiFiConnect, "WiFiConnect", 4096, NULL, 1, NULL);
    xTaskCreate(TaskCoreIOTConnect, "CoreIOTConnect", 4096, NULL, 1, NULL);
    xTaskCreate(TaskBlinkLed, "BlinkLed", 4096, NULL, 1, NULL);
    xTaskCreate(TaskSendTelemetry, "SendTelemetry", 4096, NULL, 1, NULL);
    xTaskCreate(TaskAutoLedControl, "AutoLedControl", 4096, NULL, 1, NULL);
    xTaskCreate(TaskAutoFanControl, "AutoFanControl", 4096, NULL, 1, NULL);
    xTaskCreate(TaskThingsBoardLoop, "ThingsBoardLoop", 4096, NULL, 1, NULL);
}

void loop() {
    
}
