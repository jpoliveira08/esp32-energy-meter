#include <Arduino.h>
#include <EnergyMeter.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

WiFiUDP udp;
NTPClient ntpClient(udp);

#define TOPIC "sensors/principal"  

const char* WIFI_SSID = "JP"; 
const char* WIFI_PASSWORD = "luna@067";
WiFiClient wifiClient;

const char* BROKER_MQTT = "192.168.0.108"; 
int BROKER_PORT = 1883;

PubSubClient MQTT(wifiClient);

struct ElectricalMeasurements eletricMeasurements;

double amountMeasurements;
double sumVrms;
double sumIrms;
double sumRealPower;
double sumApparentPower;
double sumPowerFactor;

bool isWifiConnected = false;

unsigned long epochTime;

void connectWifi();
void wifiConnectionTask(void* param);
void setupNTP();
void mqttReconnect(void);
void setupMQTT(void);

void setup() {
  Serial.begin(115200);
  connectWifi();
  setupNTP();
  setupMQTT();
  mqttReconnect();

  // Creating a task in the first core, to check and connect to Wifi when is needed
  xTaskCreatePinnedToCore(
    wifiConnectionTask,
    "wifiConnectionTask", // Função que será executada
    10000,
    NULL,
    2,
    NULL,
    0
  );

  setupMeasurement();
}

void loop() {
  epochTime = ntpClient.getEpochTime();

  JsonDocument doc;
  
  eletricMeasurements = makeMeasurement();
  doc["vrms"] = eletricMeasurements.vrms;
  doc["irms"] = eletricMeasurements.irms;
  doc["apparentPower"] = eletricMeasurements.apparentPower;
  doc["realPower"] = eletricMeasurements.realPower;
  doc["reactivePower"] = 0;
  doc["powerFactor"] = eletricMeasurements.powerFactor;
  doc["readAt"] = epochTime;

  String json;
  serializeJson(doc, json);
  MQTT.publish(TOPIC, (char*)json.c_str());
  /* keep-alive MQTT */    
  MQTT.loop();  

  // eletricMeasurements = makeMeasurement();
  // if (amountMeasurements == 88) {
  //   Serial.print("Vrms: ");
  //   Serial.print(sumVrms/amountMeasurements, 5);
  //   Serial.print(" Irms: ");
  //   Serial.print(sumIrms/amountMeasurements, 5);
  //   Serial.print(" Real Power: ");
  //   Serial.print(sumRealPower/amountMeasurements, 5);
  //   Serial.print(" Apparent Power: ");
  //   Serial.print(sumApparentPower/amountMeasurements, 5);
  //   Serial.print(" Power factor: ");
  //   Serial.println(sumPowerFactor/amountMeasurements, 5);
  //   amountMeasurements = 0;
  //   sumVrms = 0;
  //   sumIrms = 0;
  //   sumRealPower = 0;
  //   sumApparentPower = 0;
  //   sumPowerFactor = 0;
  // }
  // sumVrms += eletricMeasurements.vrms;
  // sumIrms += eletricMeasurements.irms;
  // sumRealPower += eletricMeasurements.realPower;
  // sumApparentPower += eletricMeasurements.apparentPower;
  // sumPowerFactor += eletricMeasurements.powerFactor;
  // amountMeasurements++;
}

void connectWifi() {
  Serial.println("Connecting");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    isWifiConnected = false;
    Serial.print(".");
    delay(500);
  }

  isWifiConnected = true;

  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
}

void wifiConnectionTask(void* param) {
  while(true) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
    }

    vTaskDelay(100);
  }
}

void setupNTP() {
  if (!isWifiConnected) {
    return;
  }

  ntpClient.begin();

  Serial.println("Waiting for firt update");

  while(!ntpClient.update()) {
    Serial.print(".");
    ntpClient.forceUpdate();
    delay(500);
  }

  Serial.println();
  Serial.println("First Update Complete");
}

void setupMQTT(void) 
{
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);        
}

void mqttReconnect(void) 
{
  String espMac = WiFi.macAddress();

    while (!MQTT.connected()) 
    {
        Serial.print("Trying to connect to the MQTT broker: ");
        Serial.println(BROKER_MQTT);

        if (MQTT.connect(espMac.c_str())) {
            Serial.println("Connected to the MQTT broker!");
        } else {
            Serial.print("Failed, code=");
            Serial.print(MQTT.state());
            Serial.println(" Trying again in 5 seconds");
            delay(5000);
        }
    }
}