#include <M5StickCPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

#define i2caddr 0x48

Adafruit_ADS1115 ads;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include "config.h"

//WIFI
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;
const char* mqtt_server = MQTT_SERVER;

//MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String stat;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    stat += (char)payload[i];
  }
  Serial.println();
  lock(stat);
}

void reconnect() {
  // Loop until it is connected
  while (!client.connected()) {

    Serial.print("Attempting MQTT connection...");
    //String clientId = "MQTT_Werkplaats";
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");

      char subscribe_topic[100];
      sprintf(subscribe_topic, "%s/status/", MQTT_TOPIC_PREFIX);
      client.subscribe(subscribe_topic);

      Serial.println("subscribed");

      //client.subscribe("tkkrlab/spacestate");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void gyro(float* gy_x, float* gy_y, float* gy_z)
{
  float gyroX = 0.0F;
  float gyroY = 0.0F;
  float gyroZ = 0.0F;
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  //M5.Lcd.setCursor(30, 40);
  //M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  //M5.Lcd.setCursor(170, 40);
  M5.Lcd.print("o/s");
  Serial.print("  gyroX : ");
  Serial.print(gyroX);
  Serial.print("  gyroY : ");
  Serial.print(gyroY);
  Serial.print("  gyroZ : ");
  Serial.println(gyroZ);
  *gy_x = map(gyroX, -200, 200, -100, 100 );
  *gy_y = map(gyroY, -200, 200, -100, 100 );
  *gy_z = map(gyroZ, -200, 200, -100, 100 );
}

void acc(float* aX, float* aY, float* aZ)
{
  float accX = 0.0F;
  float accY = 0.0F;
  float accZ = 0.0F;
  float x = 0;
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  //M5.Lcd.setCursor(30, 50);
  //M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  //M5.Lcd.setCursor(170, 50);
  //M5.Lcd.print("G");
  Serial.print("  accX : ");
  Serial.print(accX);
  Serial.print("  accY : ");
  Serial.print(accY);
  Serial.print("  accZ : ");
  Serial.println(accZ);
  *aX = accX;
  *aY = accY;
  *aZ = accZ;
}

void pit(float* pit, float* rol, float* ya)
{
  float pitch = 0.0F;
  float roll  = 0.0F;
  float yaw   = 0.0F;
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  //M5.Lcd.setCursor(30, 80);
  //M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);
  Serial.print("  pitch : ");
  Serial.print(pitch);
  Serial.print("  roll : ");
  Serial.print(roll);
  Serial.print("  yaw : ");
  Serial.println(yaw);
  *pit = pitch;
  *rol = roll;
  *ya = yaw;
}

void tempt(float* tem)
{
  static float temp = 0;
  M5.IMU.getTempData(&temp);
  //M5.Lcd.setCursor(30, 95);
  //M5.Lcd.printf("Temperature : %.2f C", temp);
  Serial.print("  temp : ");
  Serial.println(temp);
  *tem = temp;
}

void setup_imu()
{
  M5.Imu.Init();
  M5.Lcd.setRotation(3);
  //M5.Lcd.fillScreen(BLACK);
  //M5.Lcd.setTextColor(WHITE);
  //M5.Lcd.setTextSize(1);
  //M5.Lcd.setCursor(80, 15);
  //M5.Lcd.println("Data");
  //M5.Lcd.setCursor(30, 30);
  //M5.Lcd.println("  X       Y       Z");
  //M5.Lcd.setCursor(30, 70);
  //M5.Lcd.println("  Pitch   Roll    Yaw");
}

void setup_resis()
{
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void resist(int16_t* a0, int16_t* a1, int16_t* a2, int16_t* a3)
{
  int16_t adc0, adc1, adc2, adc3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  Serial.print("AIN0: "); Serial.print(adc0); Serial.println("  ");
  Serial.print("AIN1: "); Serial.print(adc1); Serial.println("  ");
  Serial.print("AIN2: "); Serial.print(adc2); Serial.println("  ");
  Serial.print("AIN3: "); Serial.print(adc3); Serial.println("  ");
  delay(1000);
  *a0 = adc0;
  *a1 = adc1;
  *a2 = adc2;
  *a3 = adc3;
}

void setup_pwm(int sda, int scl)
{
  Wire.begin(sda, scl);
  pwm.begin();         // Sends PWM signals.
  pwm.setPWMFreq(60);  // Makes servos run at 60 Hz rate.
  delay(200);
}

void lock(String msg)
{
  if (msg == "Haven't grab") {
    for (int i = 2; i < 7; i++)
    {
      servo(i, 0, 600);
    }
  }
  if (msg == "Grabed") {
    for (int i = 2; i < 7; i++)
    {
      servo(i, 0, 100);
    }
  }
}

void servo(int sv, int servomin, int servomax)
{
  pwm.setPWM(sv, servomin, servomax);
  delay(10);
}

void sent_data_f(String text, String v1, String v2, String v3, float x, float y, float z)
{
  char topic[128];
  char payload[100];
  char out[128];
  StaticJsonDocument<256> doc;
  doc[v1] = x;
  doc[v2] = y;
  doc[v3] = z;
  serializeJson(doc, out);
  if (text == "gyro/") {
    sprintf(topic, "%s/gyro/", MQTT_TOPIC_PREFIX);
  }
  else if (text == "pitch/")
  {
    sprintf(topic, "%s/pitch/", MQTT_TOPIC_PREFIX);
  }
  else if (text == "acc/")
  {
    sprintf(topic, "%s/acc/", MQTT_TOPIC_PREFIX);
  }
  sprintf(payload, out);
  client.publish(topic, payload);
}

void sent_data_i(String text, String v1, String v2, String v3, String v4, int f1, int f2, int f3, int f4)
{
  char topic[128];
  char payload[100];
  char out[128];
  StaticJsonDocument<256> doc;
  doc[v1] = f1;
  doc[v2] = f2;
  doc[v3] = f3;
  doc[v4] = f4;
  serializeJson(doc, out);
  if (text == "grasp/movement/") {
    sprintf(topic, "%s/grasp/movement/", MQTT_TOPIC_PREFIX);
  }
  sprintf(payload, out);
  client.publish(topic, payload);
}

void analog(int hpin, uint16_t* finger) {
  Wire.beginTransmission(i2caddr);
  Wire.write(0x01);
  Wire.write(hpin);
  Wire.write(0x83);
  Wire.endTransmission();
  delay(300);

  byte Analog_data[2];
  Wire.beginTransmission(i2caddr);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(i2caddr, 2);
  if (Wire.available() == 2)
  {
    Analog_data[0] = Wire.read();
    Analog_data[1] = Wire.read();
  }
  uint16_t raw_adc = Analog_data[0] + Analog_data[1];
  *finger = raw_adc;
}

void setup() {

  M5.begin();

  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(2048);

  pinMode(BUTTON_A_PIN, INPUT);


  setup_imu();
  //setup_resis();
  setup_pwm(32, 33);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  M5.Lcd.fillScreen(0xa254);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(80, 35);
  M5.Lcd.printf("CMU");
  M5.Lcd.setTextSize(2.8);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(40, 85);
  M5.Lcd.printf("Bravely Divide");


  uint16_t f1, f2, f3, f4;
  analog(0b01000010, &f1);
  analog(0b01010010, &f2);
  analog(0b01100010, &f3);
  analog(0b01110010, &f4);
  Serial.println("Analog");
  Serial.print("F1 : "); Serial.println(f1);
  Serial.print("F2 : "); Serial.println(f2);
  Serial.print("F3 : "); Serial.println(f3);
  Serial.print("F4 : "); Serial.println(f4);
  delay(500);

  //int16_t f1, f2, f3, f4;
  float gyroX, gyroY, gyroZ, accX, accY, accZ;
  //float pitch, roll, yaw;
  gyro(&gyroX, &gyroY, &gyroZ);
  acc(&accX, &accY, &accZ);

  //pit(&pitch, &roll, &yaw);

  //resist(&f1, &f2, &f3, &f4);
  //M5.Lcd.setCursor(30, 100);
  //M5.Lcd.printf(" F1       F2       F3       F4");
  //M5.Lcd.setCursor(30, 110);
  //M5.Lcd.printf(" %d       %d       %d       %d", f1, f2, f3, f4);

  sent_data_f("gyro/", "gyro_x", "gyro_y", "gyro_z", gyroX, gyroY, gyroZ);
  //sent_data_f("pitch/", "pitch", "roll", "yaw", pitch, roll, yaw);
  sent_data_f("acc/", "acc_x", "acc_y", "acc_z", accX, accY, accZ);
  sent_data_i("grasp/movement/", "finger_1", "finger_2", "finger_3", "finger_4", f1, f2, f3, f4);
  delay(100);

}
