#include <FS.h>
#include <WiFiManager.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "time.h"




#define WiFi_RESET_PIN            2
#define DHT_PIN                   5
#define DHT_TYPE                  DHT22

#define NTP_SERVER                "pool.ntp.org"

#define WiFi_CONNECTION_TIMEOUT   120
#define WiFi_STATUS_LED           4
#define WiFi_RESET_PRESS_TIME     5000
#define WiFI_CONNECTION_TIMEOUT   10000
#define DELAY_FOR_LED_BLINKING    300
#define DELAY_BETWEEN_DATA_SEND   30000
#define MQTT_RECONNECT_DELAY      5000
#define MQTT_RECONNECT_TIMER      5
#define WiFi_RECONNECT_TIMER      10



#define TEST_CP                   false // always start the configportal, even if ap found
#define TESP_CP_TIMEOUT           90 // test cp timeout
#define NON_BLOCKING_MODE         false

#define DEVICE_AP_NAME            "IoT_Testing_device_1"
#define DEVICE_AP_PASS            "12345678"

bool SHOULD_SAVE_CONFIG =         false;


WiFiManager wm;
DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient espClient;
PubSubClient client(espClient);

char mqtt_server[40];
char mqtt_port[6];
char mqtt_topic[34];

bool check_wifi_reset_pin();
void reset_wifi_config();
void led_blinking();
void wifiInfo();
void wifi_connected_successfully();
void set_wifi_for_pressing_button();
void read_wifi_and_mqtt_credentials();
void reconnect_wifi();
void dht_init();
float get_temp_data();
float get_hum_data();
void  mqtt_init();
unsigned long getTime();
void config_timestamp();


TaskHandle_t Task1;
TaskHandle_t Task2;


unsigned long previousMillis = 0;

int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
bool isPressing = false;
bool isLongDetected = false;
unsigned int counter = 0;
unsigned long timestamp = 0;
StaticJsonDocument<256> doc;
char jsonBuffer[256];


void setup() {
  Serial.begin(115200);
  pinMode(WiFi_RESET_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  Serial.print("Starting ");
  Serial.println(DEVICE_AP_NAME);

  pinMode(WiFi_STATUS_LED, OUTPUT);
  digitalWrite(WiFi_STATUS_LED, HIGH);
  dht_init();
  read_wifi_and_mqtt_credentials();
  wifi_connected_successfully();
  mqtt_init();
  config_timestamp();


  //  xTaskCreatePinnedToCore(
  //    Task1code,
  //    "Task1",
  //    10000,
  //    NULL,
  //    1,
  //    &Task1,
  //    0);
  delay(500);

  xTaskCreatePinnedToCore(
    Task2code,
    "Task2",
    10000,
    NULL,
    1,
    &Task2,
    1);
  delay(500);
}

void Task2code(void * pvParameters) {
  for (;;) {
    currentState = digitalRead(WiFi_RESET_PIN);
    Serial.println(currentState);
    if (currentState > 0) {
      counter++;
      if (counter == 10) {
        Serial.println("Reset Occur");
        counter = 0;
        digitalWrite(WiFi_STATUS_LED, LOW);
        digitalWrite(WiFi_STATUS_LED, HIGH);
        delay(5000);
        digitalWrite(WiFi_STATUS_LED, LOW);
        reset_wifi_config();
      }
    }
    delay(1000);

  }
}


//void Task2code(void * pvParameters) {
//  for (;;) {
//
//  }
//}


void loop() {
  unsigned long currentMillis = millis();
  client.loop();
//  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= WiFI_CONNECTION_TIMEOUT)) {
//    reconnect_wifi();
//  }
  int h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  int t = dht.readTemperature();
  timestamp = getTime();
  Serial.println(h);
  //  Serial.println(t);
  //  Serial.println(timestamp);
  if (!client.connected()) {
    if (timestamp != 0) {
      doc["timestamp"] = timestamp;
      doc["temp"] = t;
      doc["hum"] = h;
      serializeJson(doc, Serial);
      Serial.println();
      serializeJson(doc, jsonBuffer);
      delay(2000);
    }

  }
  delay(5000);
}

void dht_init() {
  Serial.println(F("DHT22 test!!"));
  dht.begin();
}

float get_temp_data() {
  float temperature = dht.readTemperature();
  return temperature;
}

float get_hum_data() {
  float humidity = dht.readHumidity();
  return humidity;
}

void reconnect_wifi() {
  int reconnect_wifi_counter = 0;
  Serial.println("Reconnecting to WiFi.....");
  WiFi.disconnect();
  String get_ssid = String(wm.getWiFiSSID());
  char ssid[50];
  get_ssid.toCharArray(ssid, 50);

  String get_pass = String(wm.getWiFiPass());
  char pass[50];
  get_pass.toCharArray(pass, 50);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    reconnect_wifi_counter++;
    delay(1000);
    Serial.print(".");
    if (reconnect_wifi_counter == WiFi_RECONNECT_TIMER) {
      Serial.println("\nCan't connect to WiFi!");
      Serial.println("Restart WiFi");
      ESP.restart();
    }
  }
}
void  mqtt_init() {
  Serial.println(mqtt_server);
  Serial.println(mqtt_port);
  unsigned int mqtt_trying_count = 0;
  client.setServer(mqtt_server, atoi(mqtt_port));
  while (!client.connected()) {
    mqtt_trying_count++;
    Serial.println("Connecting to MQTT....");
    delay(1000);
    if (client.connect("ESP32Client")) {
      Serial.println("MQTT Connected");
    }
    else {
      
      Serial.print("MQTT connection failed with state ");
      Serial.println(client.state());
      if(mqtt_trying_count==MQTT_RECONNECT_TIMER){
        break;
      }
      delay(1000);
    }
  }
}

void mqtt_reconnect() {
  int mqtt_reconnect_try = 0;
  Serial.println("MQTT Connection faild! Reconnecting");
  while (!client.connected()) {
    Serial.println("Attempting MQTT connecting...");
    if (client.connect(DEVICE_AP_NAME)) {
      Serial.println("MQTT Connected");
    }
    else {
      Serial.print("MQTT Connection failed, rc ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      mqtt_reconnect_try++;
      delay(MQTT_RECONNECT_DELAY);
      if (mqtt_reconnect_try == MQTT_RECONNECT_TIMER) {
        Serial.println("Can not connect to broker! Check broker address");
        break;
      }
    }
  }
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Time not sync.");
    return (0);
  }
  time(&now);
  return now;
}

void config_timestamp() {
  Serial.println();
  configTime(0, 0, NTP_SERVER);
  while (1) {
    if (!getLocalTime) {
      Serial.println("Time not sync.");
      delay(1000);
    }
    Serial.println("Time sync successfull.");
    break;
  }
}

void wifi_connected_successfully() {
  if (WiFi.status() == WL_CONNECTED) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(WiFi_STATUS_LED, LOW);
      delay(1000);
      digitalWrite(WiFi_STATUS_LED, HIGH);
      delay(1000);
    }
    Serial.println("WiFi Connected");
    wifiInfo();
    digitalWrite(WiFi_STATUS_LED, LOW);
  }
  else {
    digitalWrite(WiFi_STATUS_LED, HIGH);
    Serial.println("WiFi Not Connected");
  }
  Serial.print("Local ip: ");
  Serial.println(WiFi.localIP());

}
void saveWifiCallback() {
  Serial.println("[CALLBACK] saveCallback fired");
  led_blinking();
}

void configModeCallback (WiFiManager * myWiFiManager) {
  Serial.println("[CALLBACK] configModeCallback fired");
  for (int i = 0; i < 20; i++) led_blinking();
}

void saveMQTTConfigCallback () {
  Serial.println("Should save config");
  SHOULD_SAVE_CONFIG = true;
}

void saveParamCallback() {
  Serial.println("[CALLBACK] saveParamCallback fired");
  digitalWrite(WiFi_STATUS_LED, HIGH);
  // wm.stopConfigPortal();
}


void read_wifi_and_mqtt_credentials() {

  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json"));
    File configFile = SPIFFS.open("/config.json", "r");
    if (configFile) {
      Serial.println("Opened config file");
      size_t file_size = configFile.size();
      std::unique_ptr<char[]>buf(new char[file_size]);
      configFile.readBytes(buf.get(), file_size);
      DynamicJsonDocument json(1024);
      auto deserializeError = deserializeJson(json, buf.get());
      serializeJson(json, Serial);
      if (!deserializeError) {
        Serial.println("\nparsed json");
        strcpy(mqtt_server, json["mqtt_server"]);
        strcpy(mqtt_port, json["mqtt_port"]);
        strcpy(mqtt_topic, json["mqtt_topic"]);
      }
      else {
        Serial.println("failed to load json config");
      }
      configFile.close();
    }
  }
  else {
    Serial.println("failed to mount FS");
  }

  // The extra parameters to be configured (can be either global or just in the setup)

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", mqtt_topic, 32);
  // callbacks
  wm.setAPCallback(configModeCallback);
  wm.setSaveConfigCallback(saveWifiCallback);
  wm.setSaveConfigCallback(saveMQTTConfigCallback);
  wm.setSaveParamsCallback(saveParamCallback);

  if (NON_BLOCKING_MODE) wm.setConfigPortalBlocking(false);

  wm.setDebugOutput(true);
  wm.debugPlatformInfo();
  wm.setConfigPortalTimeout(WiFi_CONNECTION_TIMEOUT);
  wm.setBreakAfterConfig(true);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_topic);

  bool res;
  res = wm.autoConnect(DEVICE_AP_NAME, DEVICE_AP_PASS);
  if (!res) {
    Serial.println("Failed to connect or hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  }

  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());

  Serial.println("The Values in the file are: ");
  Serial.println("\tmqtt_server: " + String(mqtt_server));
  Serial.println("\tmqtt_port: " + String(mqtt_port));
  Serial.println("\tmqtt_topic: " + String(mqtt_topic));

  if (SHOULD_SAVE_CONFIG) {
    Serial.println("Saving MQTT Config");
    DynamicJsonDocument json(1024);
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_topic"] = mqtt_topic;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
    }
    serializeJson(json, Serial);
    //serializeJson(json, configFile);
    if (serializeJson(json, configFile)) {
      Serial.println("File written");
    }
    else {
      Serial.println("-file write faild");
    }
    configFile.close();
  }

}
void reset_wifi_config() {
  delay(100);
  WiFi.disconnect();
  wm.resetSettings();
  ESP.restart();
  delay(200);
  wm.setConfigPortalTimeout(WiFi_CONNECTION_TIMEOUT);
  if (!wm.startConfigPortal(DEVICE_AP_NAME, DEVICE_AP_PASS)) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
  }
  else {
    wifi_connected_successfully();
  }

}

void led_blinking() {
  digitalWrite(WiFi_STATUS_LED, LOW);
  delay(DELAY_FOR_LED_BLINKING);
  digitalWrite(WiFi_STATUS_LED, HIGH);
  delay(DELAY_FOR_LED_BLINKING);
}

void wifiInfo() {
  // can contain gargbage on esp32 if wifi is not ready yet
  Serial.println("[WIFI] WIFI INFO DEBUG");
  // WiFi.printDiag(Serial);
  Serial.println("[WIFI] SAVED: " + (String)(wm.getWiFiIsSaved() ? "YES" : "NO"));
  Serial.println("[WIFI] SSID: " + (String)wm.getWiFiSSID());
  Serial.println("[WIFI] PASS: " + (String)wm.getWiFiPass());
  Serial.println("[WIFI] HOSTNAME: " + (String)WiFi.getHostname());
}
