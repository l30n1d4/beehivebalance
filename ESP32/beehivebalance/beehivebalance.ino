/*
 * Bee Hive Balance
 * Fabio Tozzi e Marco Gardoni
 * Complete project details at https://github.com/l30n1d4/beehivebalance
*/

// Define how you're planning to connect to the internet
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true

// Your GPRS credentials (leave empty, if not needed)
const char apn[]      = "web.kenamobile.it";              // APN (example: iliad) use https://wiki.apnchanger.org
const char gprsUser[] = "";                               // GPRS User
const char gprsPass[] = "";                               // GPRS Password

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// Your WIFI credentials (leave empty, if not needed)
const char ssid[]     = "................";               // SSID WIFI
const char wifiPass[] = "................";               // WIFI Password

// Server details
// The server variable can be just a domain name or it can have a subdomain. It depends on the service you are using
const char server[]   = "example.alwaysdata.net";         // domain name: example.com, maker.ifttt.com, etc
const char resource[] = "/beehivebalance/post-data.php";  // resource path, for example: /post-data.php
const int  port = 80;                                     // server port number

// Keep this API Key value to be compatible with the PHP code provided in the project page. 
// If you change the apiKeyValue value, the PHP file /post-data.php also needs to have the same key 
String apiKeyValue = "tPmAT5Ab3j7F9";

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
// BME280 pins
#define I2C_SDA_2            18
#define I2C_SCL_2            19
// HX711 pins
#define LOADCELL_DOUT_PIN    33
#define LOADCELL_SCK_PIN     32

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>
#include <WiFi.h>
#include <soc/rtc.h>               // for downspeed clock https://github.com/bogde/HX711/issues/75
#include <HX711.h>
float HX711_CAL_FACTOR = -1980;  //You must change this factor depends on your scale,sensors and etc.

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// I2C for BME280 sensor
TwoWire I2CBME = TwoWire(1);
Adafruit_BME280 bme;
#define BME280_ADDR          0x76  // You might need to change the BME280 I2C address, in our case it's 0x76
#define SEALEVELPRESSURE_HPA (1013.25)

#if TINY_GSM_USE_GPRS
// TinyGSM Client for Internet connection
TinyGsmClient client(modem);
#endif

#define uS_TO_S_FACTOR 1000000     // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  3600        // Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37);          // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35);          // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

int8_t getBatteryLevel() {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(0x78);
  if (I2CPower.endTransmission(false) == 0
   && I2CPower.requestFrom(IP5306_ADDR, 1)) {
    switch (I2CPower.read() & 0xF0) {
    case 0xE0: return 25;
    case 0xC0: return 50;
    case 0x80: return 75;
    case 0x00: return 100;
    default: return 0;
    }
  }
  return -1;
}

float getWeight() {
  HX711 scale;
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(HX711_CAL_FACTOR); //Adjust to this calibration factor
  SerialMon.print("Reading: ");
  float weight = scale.get_units();
  if (weight < 0) {
    weight = 0.00;
  }
  SerialMon.print(weight);
  SerialMon.print(" g."); // You can change this to other type of weighing value and re-adjust the calibration factor.
  SerialMon.print(" calibration_factor: ");
  SerialMon.print(HX711_CAL_FACTOR);
  SerialMon.println();
  return weight;
}

bool getBme() {
  float t = bme.readTemperature();
  float p = (bme.readPressure() / 100.0F);
  float h = bme.readHumidity();
  float a = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if (isnan(t) || isnan(p) || isnan(h) || isnan(a)) {
    SerialMon.println("Failed to read from BME sensor!");
    return false;
  }
  SerialMon.print("Temperature = ");
  SerialMon.print(t);
  SerialMon.println(" *C");
  SerialMon.print("Pressure = ");
  SerialMon.print(p);
  SerialMon.println(" hPa");
  SerialMon.print("Approx. Altitude = ");
  SerialMon.print(a);
  SerialMon.println(" m");
  SerialMon.print("Humidity = ");
  SerialMon.print(h);
  SerialMon.println(" %");
  SerialMon.println();
  return true;
}

void setup() {
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);  // for downspeed clock https://github.com/bogde/HX711/issues/75
  
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  int8_t batt = getBatteryLevel();
  SerialMon.println("Battery level: " + batt);

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

#if TINY_GSM_USE_GPRS
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);
#endif
#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
#endif
  
  if (!bme.begin(BME280_ADDR, &I2CBME)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
  //scale.power_up();
  float weight = getWeight();
  bool okBme = getBme();
  bool connect = false;

#if TINY_GSM_USE_GPRS
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  connect = modem.gprsConnect(apn, gprsUser, gprsPass);
#endif
#if TINY_GSM_USE_WIFI
  SerialMon.print("Connecting to WIFI: ");
  SerialMon.print(ssid);
  WiFi.begin(ssid, wifiPass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SerialMon.print(".");
  }
  WiFiClient client;
  connect = true;
#endif
  if (!connect) {
    SerialMon.println(" fail");
    delay(30000);
    //reboot
    while(1);
  }
  else {
    SerialMon.println(" OK");
    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port)) {
      SerialMon.println(" fail");
    }
    else {
      SerialMon.println(" OK");
    
      // Making an HTTP POST request
      SerialMon.println("Performing HTTP POST request...");
      // Prepare your HTTP POST request data (Temperature in Celsius degrees)
      String httpRequestData = "api_key=" + apiKeyValue
                             + "&temp_ext=" + String("25.00")
                             + "&temp_int=" + String(bme.readTemperature())
                             + "&humidity=" + String(bme.readHumidity())
                             + "&pressure=" + String(bme.readPressure()/100.0F)
                             + "&altitude=" + String(bme.readAltitude(SEALEVELPRESSURE_HPA))
                             + "&weight="   + String(weight)
                             + "&battery="  + String(getBatteryLevel());
    
      client.print(String("POST ") + resource + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(httpRequestData.length());
      client.println();
      client.println(httpRequestData);

      unsigned long timeout = millis();
      while (client.connected() && millis() - timeout < 10000L) {
        // Print available data (HTTP response from server)
        while (client.available()) {
          char c = client.read();
          SerialMon.print(c);
          timeout = millis();
        }
      }
      SerialMon.println();
    
      // Close client and disconnect
      client.stop();
      SerialMon.println(F("Server disconnected"));
#if TINY_GSM_USE_GPRS
      modem.gprsDisconnect();
      SerialMon.println(F("GPRS disconnected"));
#endif
#if TINY_GSM_USE_WIFI
      WiFi.disconnect();
      //modem.networkDisconnect();
      SerialMon.println(F("WiFi disconnected"));
#endif
    }
  }
  delay(60000);
  // Put ESP32 into deep sleep mode (with timer wake up)
  esp_deep_sleep_start();
}
