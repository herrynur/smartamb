#include <Arduino.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>

// led
#define ledR 2

// TDS Sensor
#define tds 39
#define VREF_TDS 3.3
#define SCOUNT 30

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
int tdsValue = 0;
float temperature = 25;

int getMedianNum(int bArray[], int iFilterLen);
void tdsRead();
void sendData(int time_);

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}
// End TDS Sensor

// PH
#define PH_PIN 34
float voltage, phValue, _temperature = 25;
DFRobot_PH ph;

void phRead()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U)
  { // time interval: 1s
    timepoint = millis();
    voltage = analogRead(PH_PIN) / 4096.0 * 3300;
    phValue = ph.readPH(voltage, _temperature);
  }
  ph.calibration(voltage, _temperature);
}
// end pH

// DO Sensor
#define DO_PIN 35
#define VREF 3300    // VREF (mv)
#define ADC_RES 4096 // ADC Resolution
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25) // Current water temperature ℃, Or temperature sensor function
#define CAL1_V (1600)  // mv
#define CAL1_T (25)    //℃
#define CAL2_V (1300)  // mv
#define CAL2_T (15)    //℃
float Do_result;

const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

void do_read();

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c);
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
  #if TWO_POINT_CALIBRATION == 0
    uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  #else
    uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  #endif
}
// End DO

//Wifi and mqtt Setup
// Setting pass dan ssid wifi disini
#define ledwifi 13
const char *ssid = "Asus_X01BDA";
const char *password = "heri1234567";

// mqtt
const char *mqtt_server = "broker.hivemq.com";
#define MQTT_USERNAME ""
#define MQTT_KEY ""

// Setting id disini, id disini harus sama dengan yang di Android
String topic = "smartambak/tambak1";
//Topic utama adalah "smartambak/tambak1"

//Topic untuk control akuator
// Button 1 -> smartambak/tambak1/B1
// Button 2 -> smartambak/tambak1/B2
// Button 3 -> smartambak/tambak1/B3
// Button 4 -> smartambak/tambak1/B4

String topicB1;
String topicB2;
String topicB3;
String topicB4;

String dataKirim = "-,-,-,-";

// wifi client
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);

// connectt wifi
void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(ledwifi, HIGH);
    delay(100);
    digitalWrite(ledwifi, LOW);
    Serial.print(".");
  }

  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(ledwifi, HIGH);
}

// Reconnect saat putus
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  //Set topic akuator
  topicB1 = topic + "/B1";
  topicB2 = topic + "/B2";
  topicB3 = topic + "/B3";
  topicB4 = topic + "/B4";
  Serial.begin(115200);
  pinMode(tds, INPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledwifi, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  ph.begin();
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  tdsRead();
  phRead();
  do_read();
  sendData(2500);
}

void do_read()
{
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  Do_result = readDO(ADC_Voltage, Temperaturet)*0.001;
  //Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
  //Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  //Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
  // Serial.print("DO:\t" + String(Do_result) + "\t");
  // Serial.println("DO:\t" + String(readDO(ADC_Voltage, Temperaturet)) + "\t");
  //delay(1000);
}

void tdsRead()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(tds);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
    {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF_TDS / 4096.0;
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      float compensationVoltage = averageVoltage / compensationCoefficient;
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

      // Serial.print("TDS Value:");
      // Serial.print(tdsValue, 0);
      // Serial.print(" ppm | ph : ");
      // Serial.println(phValue, 2);
    }
  }
}

void sendData(int time_)
{
  String dataKirim = "-,-,-,-";
  dataKirim = String(temperature) + "," + String(phValue) + "," + String(tdsValue) + "," + String(Do_result);
  static unsigned long refresh = millis();
  if (millis() - refresh > time_)
  {
    refresh = millis();
    Serial.print("TDS Value => ");
    Serial.print(tdsValue, 0);
    Serial.print(" ppm | ph => ");
    Serial.print(phValue, 2);
    Serial.print(" | DO => ");
    Serial.print(Do_result, 2);
    Serial.println(" mg/L");
    client.publish((char *)topic.c_str(), (char *)dataKirim.c_str());
    digitalWrite(ledR, HIGH);
    delay(100);
    digitalWrite(ledR, LOW);
    delay(100);
  }
}

//Program utama saat menerima data dari mqtt
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Setting untuk Butoon1 dan Button2
  //Relay 1
  if (strcmp(topic, (char*) topicB1.c_str()) == 0)
  {
    if ((char)payload[0] == '1') {
      //Saat menerima data 1 maka relay 1 menyala
      Serial.print("Data B1 : ");
      Serial.println(payload[0]);
      digitalWrite(2, LOW);
    }
    else
    {
      //Saat menerima data 0 maka relay 1 mati
      Serial.print("Data B1 : ");
      Serial.println(payload[0]);
      digitalWrite(2, HIGH);
    }
  }
  //Relay2
  else if (strcmp(topic, (char*) topicB2.c_str()) == 0)
  {
    if ((char)payload[0] == '1') {
      Serial.print("Data B2 : ");
      Serial.println(payload[0]);
      digitalWrite(2, LOW);
    }
    else
    {
      Serial.print("Data B2 : ");
      Serial.println(payload[0]);
      digitalWrite(2, HIGH);
    }
  }
  //Bisa ditambah porgram yang sama untuk relay 3 dan 4 dengan mengganti topic
}