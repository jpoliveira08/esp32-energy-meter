#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define CLOCKRATE 80000000 /* Hz */
#define TIMERDIVIDER 4

#define OVER_SAMPLE_RATIO (16)
#define CYCLES (20)
#define NSAMPLES (OVER_SAMPLE_RATIO * CYCLES) // 321

#define VOLTAGE_ADC_PIN (34)
#define CURRENT_ADC_PIN (35)

#define RMS_TOPIC "measurements/rms"

volatile int sampleCount = NSAMPLES;
volatile int voltageSamples[NSAMPLES];
volatile int currentSamples[NSAMPLES];

hw_timer_t *My_timer = NULL;

WiFiUDP udp;
NTPClient ntpClient(udp);

const char* WIFI_SSID = "nomeWiFi"; 
const char* WIFI_PASSWORD = "senhaWifi";
WiFiClient wifiClient;

const char* BROKER_MQTT = "ip_host_broker"; 
int BROKER_PORT = 1883;

PubSubClient MQTT(wifiClient);

struct ElectricalMeasurements {
  double vrms;
  double irms;
  double realPower;
  double apparentPower;
  double powerFactor;
};

struct ElectricalMeasurements measurements;
struct ElectricalMeasurements makeMeasurement();

void setupMeasurement();

bool isWifiConnected = false;

unsigned long epochTime;

int countRmsMeasurements = 0;
double sumVoltageToSend = 0.0;
double sumCurrentToSend = 0.0;
double sumRealPowerToSend = 0.0;

const float voltageCalibration = 282.5;
const float currentCalibration = 30.5;

void connectWifi();
void checkConnectionsTask(void* param);
void setupNTP();
void mqttReconnect(void);
void setupMQTT(void);

void setup() {
  connectWifi();
  setupNTP();
  setupMQTT();

  // Creating a task in the first core, to check and connect to Wifi when is needed
  xTaskCreatePinnedToCore(
    checkConnectionsTask,
    "checkConnectionsTask",
    10000,
    NULL,
    2,
    NULL,
    0
  );

  setupMeasurement();
}

void loop() {
  measurements = makeMeasurement();

  if (countRmsMeasurements < 100) {
    sumVoltageToSend += measurements.vrms;
    sumCurrentToSend += measurements.irms;
    sumRealPowerToSend += measurements.realPower;

    countRmsMeasurements++;
    
    return;
  }

  JsonDocument doc;

  float VrmsConverted = sumVoltageToSend / 100.0;
  float IrmsConverted = sumCurrentToSend / 100.0;
  float realPowerConverted =sumRealPowerToSend / 100.0;
  float apparentPowerConverted = VrmsConverted * IrmsConverted;
  float powerFactorConverted = realPowerConverted / apparentPowerConverted;

  doc["voltage"] = VrmsConverted;
  doc["current"] = IrmsConverted;
  doc["apparentPower"] = apparentPowerConverted;
  doc["realPower"] = realPowerConverted;
  doc["powerFactor"] = powerFactorConverted;
  doc["readAt"] = ntpClient.getEpochTime();

  String json;
  serializeJson(doc, json);
  MQTT.publish(RMS_TOPIC, (char*) json.c_str());

  /* keep-alive MQTT */    
  MQTT.loop();

  sumVoltageToSend = 0;
  sumCurrentToSend = 0;
  sumRealPowerToSend = 0;
  countRmsMeasurements = 0;
}

void connectWifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    isWifiConnected = false;
    delay(500);
  }

  isWifiConnected = true;
}

void checkConnectionsTask(void* param) {
  while(true) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
    }

    if (!MQTT.connected()) {
      mqttReconnect();
    }
    vTaskDelay(100);
  }
}

void setupNTP() {
  if (!isWifiConnected) {
    return;
  }

  ntpClient.begin();

  while(!ntpClient.update()) {
    ntpClient.forceUpdate();
    delay(500);
  }
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
        if (!MQTT.connect(espMac.c_str())) {
          delay(5000);
        }
    }
}

/**
 * Timer Interrupt Service Routine (ISR)
*/
void IRAM_ATTR onTimer() {
  if ((sampleCount >= 0) && (sampleCount < (NSAMPLES))) {
    voltageSamples[sampleCount] = analogRead(VOLTAGE_ADC_PIN);
    currentSamples[sampleCount] = analogRead(CURRENT_ADC_PIN);

    sampleCount++;
  }
}


void setupMeasurement() {
  My_timer = timerBegin(
    1, 
    TIMERDIVIDER,
    true 
  );

  // when the timer interrupt happens, the function onTimer() will be called
  timerAttachInterrupt(My_timer, &onTimer, true);

  float measureRatePerInterval = 1.0 / ( 60.0 * OVER_SAMPLE_RATIO);

  // Calculates the amount of time between interrupts ~ 20833
  int amountTimeBetweenInterruption = (int)( measureRatePerInterval * CLOCKRATE / TIMERDIVIDER + 0.5);

  timerAlarmWrite(My_timer, amountTimeBetweenInterruption, true);

  // Enable Timer with interrupt (Alarm Enable)
  timerAlarmEnable(My_timer);
}

void readAnalogSamples() {
  int waitDelay = 17 * CYCLES;
  sampleCount = 0; // triggers the ISR to start reading the samples

  delay(waitDelay); // 340 ms

  // After the delay NSAMPLES sampleCount(320) == NSAMPLES (320)
  // sampleCount increases in each interruption
  if (sampleCount != NSAMPLES) {
    Serial.print("ADC processing is not working.");
  }

  timerWrite(My_timer, 0);
}

struct ElectricalMeasurements measureRms(int* voltageSamples, int* currentSamples, int nsamples) {
  struct ElectricalMeasurements eletricMeasurements;

  int offsetVoltage = 1851;
  int offsetCurrent = 1848;

  float sumVoltage = 0;
  float sumCurrent = 0;
  float sumInstantaneousPower = 0;
  for (int i = 0; i < nsamples; i++) {
    int y_voltageNoOffset = voltageSamples[i] - offsetVoltage;
    int y_currentNoOffset = currentSamples[i] - offsetCurrent;

    float y_voltage = ((float)y_voltageNoOffset) * voltageCalibration * (3.3/4096.0);
    float y_current = ((float)y_currentNoOffset) * currentCalibration * (3.3/4096.0);
    float y_instantaneousPower = y_voltage * y_current;
  
    sumVoltage += y_voltage * y_voltage;
    sumCurrent += y_current * y_current;
    sumInstantaneousPower += y_instantaneousPower;
  }

  float ym_voltage = sumVoltage / (float) nsamples;
  float ym_current = sumCurrent / (float) nsamples;
  float ym_realPower = sumInstantaneousPower / (float) nsamples;

  float vrmsSquared = sqrt(ym_voltage);
  float irmsSquared = sqrt(ym_current);

  float Vrms = vrmsSquared;
  float Irms = irmsSquared;
  float realPower = ym_realPower;

  eletricMeasurements.vrms = Vrms;
  eletricMeasurements.irms = Irms;
  eletricMeasurements.realPower = realPower;

  return eletricMeasurements;
}

struct ElectricalMeasurements makeMeasurement() {
  struct ElectricalMeasurements eletricMeasurements;

  readAnalogSamples();
  if (sampleCount == NSAMPLES) {
    eletricMeasurements = measureRms((int*) voltageSamples, (int*) currentSamples, NSAMPLES);
  }

  return eletricMeasurements;
}