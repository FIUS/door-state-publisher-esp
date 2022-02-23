#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//#define DEBUG

#define WIFI_SSID "my ssid"
#define WIFI_PASSWORD "my pw"

#define MQTT_DEVICE_NAME "FIUS-DOOR-STATE-PUBLISHER-ESP"
#define MQTT_SERVER "my server"
#define MQTT_PORT 1883
#define MQTT_USER "my user"
#define MQTT_PASS "my pw"
#define MQTT_TOPIC "my topic"
#define MQTT_RETAIN true

#define SENSOR_PIN 14 //D5 = GPIO 14
#define LED_PIN LED_BUILTIN

//times in msclientclient
#define LED_PATTERN_WIFI_CONN_ON_TIME 1000
#define LED_PATTERN_WIFI_CONN_OFF_TIME 1000
#define LED_PATTERN_MQTT_CONN_ON_TIME 2000
#define LED_PATTERN_MQTT_CONN_OFF_TIME 2000
#define LED_PATTERN_PUBLISH_ON_TIME 500
#define LED_PATTERN_STARTUP_ON_TIME 3000
#define SERIAL_STATE_INFORM_PERIOD 5000
#define REBOOT_AFTER_DISCONNECTED_FROM_WIFI_FOR 300000 //5 minutes

#define SENSOR_DEBOUNCE_LOOP_COUNT 10

//Use this file to overwrite any settings from above
#include "settings.h"

enum class State { WifiConnecting, MqttConnecting, Running };

State current_state;
State previous_state;
long current_time;

boolean blink_state;
long blink_last_switch;

long last_serial_state_inform;

long last_wifi_connection;
int current_wifi_status;

int current_mqtt_state;

boolean current_sensor_state;
boolean last_sent_sensor_state;
int sensor_state_debounce_count;

WiFiClient wifiClient = WiFiClient();
PubSubClient mqttClient = PubSubClient(wifiClient);

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  Serial.println("[BOOTUP] ESP8266 bootup.");
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

#ifdef DEBUG
  Serial.print("[WIFI] Connecting to wifi ");
  Serial.print(WIFI_SSID);
  Serial.print(" with password ");
  Serial.print(WIFI_PASSWORD);
  Serial.println(".");
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

#ifdef DEBUG
  Serial.print("[MQTT] Setting mqtt server to ");
  Serial.print(MQTT_SERVER);
  Serial.print(" and port to ");
  Serial.print(MQTT_PORT);
  Serial.println(".");
#endif

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  current_state = State::WifiConnecting;
  current_time = 0;
  blink_state = false;
  blink_last_switch = 0;
  last_serial_state_inform = -SERIAL_STATE_INFORM_PERIOD;
  last_wifi_connection = 0;
  sensor_state_debounce_count = 0;

  //signal bootup
  digitalWrite(LED_PIN, LOW);
  delay(LED_PATTERN_STARTUP_ON_TIME);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("[BOOTUP] setup done.");
}

void manageTime() {
  long new_current_time = millis();
  if(new_current_time < current_time) {
    //handle potential overflow
    blink_last_switch = new_current_time + (blink_last_switch - current_time);
    last_serial_state_inform = new_current_time + (last_serial_state_inform - current_time);
    last_wifi_connection = new_current_time + (last_wifi_connection - current_time);
  }
  current_time = new_current_time;
}

void manageLED() {
  if (current_state != previous_state) {
    blink_state = false;
    blink_last_switch = current_time;
  }

  long target_time;
  if(current_state == State::WifiConnecting) {
    if(blink_state) {
      target_time = blink_last_switch + LED_PATTERN_WIFI_CONN_ON_TIME;
    } else {
      target_time = blink_last_switch + LED_PATTERN_WIFI_CONN_OFF_TIME;
    }
  } else if (current_state == State::MqttConnecting ) {
    if(blink_state) {
      target_time = blink_last_switch + LED_PATTERN_MQTT_CONN_ON_TIME;
    } else {
      target_time = blink_last_switch + LED_PATTERN_MQTT_CONN_OFF_TIME;
    }
  } else {
    if(blink_state) {
      target_time = blink_last_switch + LED_PATTERN_PUBLISH_ON_TIME;
    } else {
      target_time = current_time; //Always in the future, because we don't want to turn on the LED
    }
  }

  if(target_time < current_time) {
    blink_state = ! blink_state;
    blink_last_switch = current_time;
  }

  digitalWrite(LED_PIN, !blink_state);
}

void informOverSerial() {
  long target_time;
  target_time = last_serial_state_inform + SERIAL_STATE_INFORM_PERIOD;
  if(target_time < current_time || current_state != previous_state) {
    String state_info = "";

    switch (current_state) {
      case State::WifiConnecting:
        state_info = "WifiConnecting. Wifi Status: ";
        state_info.concat(current_wifi_status);
        state_info.concat("; Time until reset: ");
        state_info.concat((last_wifi_connection + REBOOT_AFTER_DISCONNECTED_FROM_WIFI_FOR - current_time)/1000);
        state_info.concat("s");
        break;
      case State::MqttConnecting:
        state_info = "MqttConnecting. MQTT State: ";
        state_info.concat(current_mqtt_state);
        break;
      case State::Running:
        state_info = "Running";
        break;
    }

    Serial.println("[State] Current State: " + state_info);
    last_serial_state_inform = current_time;
  }
}

void manageWifi() {
  current_wifi_status = WiFi.status();
  if(current_wifi_status == WL_CONNECTED) {
    if(current_state == State::WifiConnecting) {
      current_state = State::MqttConnecting;
    }
    last_wifi_connection = current_time;
  } else {
    if(current_state != State::WifiConnecting) {
      Serial.print("[WIFI] Lost Wifi connection. State: ");
      Serial.println(current_wifi_status);
    }
    current_state = State::WifiConnecting;

    if(last_wifi_connection + REBOOT_AFTER_DISCONNECTED_FROM_WIFI_FOR < current_time) {
      Serial.println("[WIFI] No wifi for to long. Resetting.");
      Serial.flush();
      ESP.restart();
    }
  }
}

void manageMqtt() {
  current_mqtt_state = mqttClient.state();

  if(mqttClient.connected()) {
    if(current_state == State::MqttConnecting) {
      current_state = State::Running;
    }
  } else {
    if(current_state == State::Running) {
      Serial.print("[MQTT] Lost MQTT connection. State: ");
      Serial.println(current_mqtt_state);
      current_state = State::MqttConnecting;
    } else if (current_state == State::MqttConnecting) {
      mqttClient.connect(MQTT_DEVICE_NAME, MQTT_USER, MQTT_PASS, MQTT_TOPIC, 0, MQTT_RETAIN, "closed");
    }
  }
}

void sendMqttMessage(boolean sensor_state) {
  Serial.print("[Sensor] Change detetced. New state: ");
  Serial.println(sensor_state);
  blink_state = true;
  blink_last_switch = current_time;
  if(sensor_state) {
    mqttClient.publish(MQTT_TOPIC, "open", true);
  } else {
    mqttClient.publish(MQTT_TOPIC, "closed", true);
  }
}

void loop() {
  manageTime();
  manageWifi();
  manageMqtt();
  manageLED();
  informOverSerial();

  // Begin of actual logic
  current_sensor_state = digitalRead(SENSOR_PIN);
  if(current_sensor_state != last_sent_sensor_state) {
    sensor_state_debounce_count += 1;
    if (sensor_state_debounce_count > SENSOR_DEBOUNCE_LOOP_COUNT) {
      sendMqttMessage(current_sensor_state);
      last_sent_sensor_state = current_sensor_state;
      sensor_state_debounce_count = 0;
    }
  } else {
    sensor_state_debounce_count = 0;
  }
  // End of actual logic

  mqttClient.loop();
  delay(20);
  previous_state = current_state;
}
