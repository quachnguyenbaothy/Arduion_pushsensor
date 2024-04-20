
/*...............ESP8266 CONNECT FIREBASE..................*/
//Thư viện Esp8266 và firebase
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#include <FirebaseESP8266.h>
#include <FirebaseArduino.h>
#endif
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

//gps
TinyGPSPlus gps;
#define gpsRxPin D1
#define gpsTxPin D2
SoftwareSerial SerialGPS(gpsTxPin, gpsRxPin);
float latitude, longitude ;

// Wifi và password kết nối
#define WIFI_SSID "giamy"
#define WIFI_PASSWORD "21122009"

// Khai báo địa chỉ API Key trên firebase
#define API_KEY "AIzaSyBtbt86NZAGdha9EibXs5N2UXdtRCJ25EA"

// DATABASE URL
#define DATABASE_URL "tudiendict-default-rtdb.firebaseio.com"

// Email-password đã được add vao firebase
#define USER_EMAIL "maitronghuu@gmail.com"
#define USER_PASSWORD "15101984MaiTrongHuu@"

// Khai báo cơ sở dữ liệu
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
unsigned long count = 0;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
WiFiMulti multi;
#endif

/*...............ESP8266+ADAFRUIT+NPK SENSOR..................*/

//Thư viện oled
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Thư viện Esp8266 và Adafruit
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//Thư viện kết nối NPK
#include <SoftwareSerial.h>
#include <ModbusMaster.h>

// User và pass wifi
#define WLAN_SSID       "giamy"
#define WLAN_PASS       "21122009"

// Setup tài khoản trên Adafruit
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "maitronghuu"
#define AIO_KEY         "add41633d1e2483b8c9c31a6ec5324aa"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//Cài đặt đường dẫn feed trên Adafruit
Adafruit_MQTT_Publish Nito = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Nito");
Adafruit_MQTT_Publish Photpho = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Photpho");
Adafruit_MQTT_Publish Kali = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Kali");
Adafruit_MQTT_Publish PH = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/PH");
Adafruit_MQTT_Publish EC = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/EC");

// Thiết lập nguồn cấp dữ liệu có tên 'onoff'
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");


//Chân kết nối UART với NPK
#define SSerialRX         D5
#define SSerialTX         D6

// Cài đặt cổng kết nối cảm biến NPK
SoftwareSerial mySerial (SSerialRX, SSerialTX);
ModbusMaster node;

float nito, photpho, kali, ph, ec ; // Smart water sensor PH/ORP
float result;

void setup() {

  //gps
  Serial.begin(115200);
  SerialGPS.begin(9600);
  mySerial.begin(9600);

  node.begin(1, mySerial);
  node.begin(2, mySerial);
  /*...............Esp8266 Connect firebse..................*/

  //Ket noi firebase


  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  // Chỉ định API Key
  config.api_key = API_KEY;

  // Chỉ định email và user đăng nhập
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Chỉ định địa chỉ database
  config.database_url = DATABASE_URL;

  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);

  //Kết nối sai sẽ chuyển sang kết nối lại
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(5);

  /*...............Esp8266 Connect NPK+Oled..................*/

  //Cài đặt kết nối wifi
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);

    // Cài đặt MQTT để điều khiển feed trên adafruit.
    mqtt.subscribe(&onoffbutton);

    //Cài đặt hiển thị màn hình oled
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    delay(500);
    display.clearDisplay();
    //    display.setCursor(25, 15);
    //    display.setTextSize(1);
    //    display.setTextColor(WHITE);
    //    display.println(" NPK Sensor");
    //    display.setCursor(25, 35);
    //    display.setTextSize(1);
    //    display.print("Initializing");
    //    display.display();
    //    delay(3000);
  }
  Serial.println("Start!");
}

void GPS_data() {
  float latitude, longitude;
  while (SerialGPS.available() > 0)
    if (gps.encode(SerialGPS.read()))
    {
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print("Latitude : ");
        Serial.println(latitude, 6);
        Serial.print("Longitude : ");
        Serial.println(longitude, 6);

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 25);
        display.print("Lat: ");
        display.print(latitude, 6);
        display.setCursor(0, 25);
        display.print("Long: ");
        display.print(longitude, 6);
        display.display();
        delay(3000);

      }
    }
}

void loop() {

  // Mở kết nối MQTT
  MQTT_connect();
  //Upload data web
  dataAdafruit();
    //ket noi gps
  GPS_data();
  //Upload dữ liệu NPK lên firebase
  dataFirebase();


}

//Hàm Kết nối MQTT
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(500);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

//Hàm kết nối NPK-OLED
void dataAdafruit() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
    }
  }

  //Lấy giá trị cảm biến NPK,PH,EC
  uint16_t data[6];
  float nito, photpho, kali, ph, ec; // Smart water sensor PH/ORP
  float result;
  //===============================================
  Serial.println("Read Data NPK SENSOR: ID = 1");
  node.begin(1, mySerial);
  delay(300);

  // Đọc giá trị nito
  result = node.readHoldingRegisters(0x001E, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    nito = float(data[0]);
    Serial.print("Nito Value: ");
    Serial.print(nito);
    Serial.println("mg/Kg");
  }
  delay(500);
  // Đọc giá trị photpho
  result = node.readHoldingRegisters(0x001F, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    photpho = float(data[0]);
    Serial.print("Photpho Value: ");
    Serial.print(photpho);
    Serial.println("mg/Kg");
  }
  delay(200);

  // Đọc giá trị kali
  result = node.readHoldingRegisters(0x0020, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    kali = float(data[0]);
    Serial.print("Kali Value: ");
    Serial.print(kali);
    Serial.println("mg/Kg");
  }
  delay(200);
  Serial.println();

  //===============================================
  // doc cam bien 4 trong 1
  Serial.println("Read Data 4 in 1 SENSOR: ID = 2");
  node.begin(2, mySerial);
  delay(300);

  // Đọc giá trị ph
  result = node.readHoldingRegisters(0x0003, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    ph = float((data[0]) / 10);
    Serial.print("pH Value: ");
    Serial.print(ph);
  }

  // Đọc giá trị ec
  result = node.readHoldingRegisters(0x0002, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    ec = float((data[0]) / 1000);
    Serial.print("EC Value: ");
    Serial.print(ec);
    Serial.println("dS/m");
  }
  delay(200);

  Serial1.println();
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 15);
  display.print("N: ");
  display.print(nito);
  display.setTextSize(1);
  display.print(" mg/kg");

  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("P: ");
  display.print(photpho);
  display.setTextSize(1);
  display.print(" mg/kg");

  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("K: ");
  display.print(kali);
  display.setTextSize(1);
  display.print(" mg/kg");
  display.display();

  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print("EC: ");
  display.print(ec);
  display.setTextSize(1);
  display.print(" dS/m");
  display.display();

  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print("pH: ");
  display.print(ph);
  display.setTextSize(1);
  display.display();

  if (! Nito.publish(nito)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  if (! Photpho.publish(photpho)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  if (! Kali.publish(kali)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  if (! PH.publish(ph)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  if (! EC.publish(ec)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
}

//Hàm upload dữ liệu NPK lên firebase
void dataFirebase() {

  //float latitude = 9.123456;  // Kinh độ
  // float longitude = 105.654321;  // Vĩ độ

  float latitude, longitude;
 // bool gps_send = false;

  while (SerialGPS.available() > 0) {

    if (gps.encode(SerialGPS.read()))
    {

      if (gps.location.isValid())
      {
        //gps_send = true;

        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print("Latitude : ");
        Serial.println(latitude);
        Serial.print("Longitude : ");
        Serial.println(longitude);

      }
    }
  }
//  delay(200);
//  if (gps_send == false) {
//    Serial.println("gps is not started");
//    return;
//  }

  //Khai bao bien cho cam bien NPK
  uint16_t data[10];
  float nito, photpho, kali, ph, ec; // Smart water sensor PH/ORP
  float result;

  Serial.println("Read Data NPK SENSOR: ID = 1");
  node.begin(1, mySerial);
  delay(300);

  // Đọc giá trị nito
  result = node.readHoldingRegisters(0x001E, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    nito = float(data[0]);
    Serial.print("Nito Value: ");
    Serial.print(nito);
    Serial.println("mg/Kg");
  }
  delay(500);
  // Đọc giá trị photpho
  result = node.readHoldingRegisters(0x001F, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    photpho = float(data[0]);
    Serial.print("Photpho Value: ");
    Serial.print(photpho);
    Serial.println("mg/Kg");
  }
  delay(200);

  // Đọc giá trị kali
  result = node.readHoldingRegisters(0x0020, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    kali = float(data[0]);
    Serial.print("Kali Value: ");
    Serial.print(kali);
    Serial.println("mg/Kg");
  }

  // doc cam bien 4 trong 1
  Serial.println("Read Data 4 in 1 SENSOR: ID = 2");
  node.begin(2, mySerial);
  delay(300);

  // Đọc giá trị ph
  result = node.readHoldingRegisters(0x0003, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    ph = float((data[0]) / 10);
    Serial.print("pH Value: ");
    Serial.print(ph);
  }

  // Đọc giá trị ec
  result = node.readHoldingRegisters(0x0002, 2);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data[0] = node.receive();
    ec = float((data[0]) / 1000);
    Serial.print("EC Value: ");
    Serial.print(ec);
    Serial.println("dS/m");
  }
  delay(200);


  if (Firebase.ready() && (millis() - sendDataPrevMillis > 60000 || sendDataPrevMillis == 0))
  {
    sendDataPrevMillis = millis();

    Firebase.setTimestamp(fbdo, "/timestamp");
    if (fbdo.httpCode() == FIREBASE_ERROR_HTTP_CODE_OK)
    {
      String timestamp = String(fbdo.to<int>());
      String nitoPath = "/data/" + timestamp + "/n";
      String kaliPath = "/data/" + timestamp + "/k";
      String photphoPath = "/data/" + timestamp + "/p";
      String phPath = "/data/" + timestamp + "/ph";
      String ecPath = "/data/" + timestamp + "/ec";
      String latPath = "/data/" + timestamp + "/lat";
      String longPath = "/data/" + timestamp + "/long";


      Serial.printf("Set Nito... %s\n", Firebase.setFloat(fbdo, nitoPath, nito) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set Photpho... %s\n", Firebase.setFloat(fbdo, photphoPath, photpho) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set Kali... %s\n", Firebase.setFloat(fbdo, kaliPath, kali) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set ph... %s\n", Firebase.setFloat(fbdo, phPath, ph) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set ec... %s\n", Firebase.setFloat(fbdo, ecPath, ec) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set lat... %s\n", Firebase.setFloat(fbdo, latPath, latitude) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set long... %s\n", Firebase.setFloat(fbdo, longPath, longitude) ? "ok" : fbdo.errorReason().c_str());
      delay(500);
      Serial.printf("Get Nito... %s\n", Firebase.getFloat(fbdo, nitoPath) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());
      Serial.printf("Get Photpho... %s\n", Firebase.getFloat(fbdo, photphoPath) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());
      Serial.printf("Get Kali... %s\n", Firebase.getFloat(fbdo, kaliPath) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());
      Serial.printf("Get ph... %s\n", Firebase.getFloat(fbdo, phPath) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());
      Serial.printf("Get ec... %s\n", Firebase.getFloat(fbdo, ecPath) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());
      Serial.printf("Get lat... %s\n", Firebase.getFloat(fbdo, latPath) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());
      Serial.printf("Get long... %s\n", Firebase.getFloat(fbdo, longPath) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());


      Serial.print("Latitude : ");
      Serial.println(latitude);
      Serial.print("Longitude : ");
      Serial.println(longitude);



    }
    count++;
  }
}
