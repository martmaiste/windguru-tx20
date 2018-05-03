/* 
 *  Technoline TX20 Wind Sensor on ESP8266 for sending the wind data to Windguru
 *  Supports OLED display
 *  
 *  by Märt Maiste, Taaralabs
 *  2018-05-02
 *  
 *  Credits - https://github.com/bunnyhu/ESP8266_TX20_wind_sensor
 *  Windguru Station API - https://stations.windguru.cz/upload_api.php
 *  
 *  OLED connected to default I2C pins
 *  
 *  TX20 RJ11 -> ESP PIN
 *  Brown TxD -> Any Digital pin (in code DATAPIN)
 *  Red Vcc -> 3.3V
 *  Green DTR -> Ground
 *  Yellow GND -> Ground
 *
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <MD5Builder.h>
#include <U8g2lib.h>
#include "Secrets.h"

#define DATAPIN D3                    // TX20 digital data pin (brown)
#define DEBUG 1
#define OLED 1                        // OLED is connected

// define your credentials in Secrets.h
/*
const char* ssid = "***";
const char* pass = "***";

const char* host = "www.windguru.cz";
const char* uid = "***";
const char* apipass = "***";
*/

volatile boolean TX20IncomingData = false;

const unsigned long interval = 60000; // calculate wind speed and direction averages and send data to Windguru every 60 seconds
unsigned long interval_count;         // interval counter

char* dir[] = {                       // human readable wind directions are only shown on the OLED
  "N",
  "NNE",
  "NE",
  "ENE",
  "E",
  "ESE",
  "SE",
  "SSE",
  "S",
  "SSW",
  "SW",
  "WSW",
  "W",
  "WNW",
  "NW",
  "NNW",
};

                    // wind speed is stored and calculated in m/s
float wind_now = 0; // last wind speed reading from the sensor
float wind_max = 0; // maximum wind speed reading in the interval
float wind_min = 0; // minimum
float wind_sum = 0; // sum of all wind speed readings in the interval
float wind_num = 0; // count of how many readings in the interval
float wind_avg = 0; // calculated average wind speed
int   wind_dir = 0; // same with the wind direction
int   dir_now  = 0; // because the wind vane has 16 positions
int   dir_sum  = 0; // but the direction flucutates a little
int   dir_num  = 0; // calculating the arverage will give more realistic info
char* quarter  = ""; // human readable reading (N, SSW etc)

#ifdef OLED
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
#endif

WiFiUDP ntpUDP;      // epoch received from NTP is used for Windguru API salt
NTPClient ntp(ntpUDP, "europe.pool.ntp.org", 7200, 60000);

MD5Builder _md5;     // Windguru API requires some MD5 hashing

String md5(String str) {
  _md5.begin();
  _md5.add(String(str));
  _md5.calculate();
  return _md5.toString();
}

void isTX20Rising() {
  if (!TX20IncomingData) {
    TX20IncomingData = true;
  }  
}

void setup() {
  pinMode(DATAPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DATAPIN), isTX20Rising, RISING);
  
  Serial.begin(115200);
  Serial.println("TX20 ESP8266 Windguru");
  
  #ifdef OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFontMode(0);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
    
  u8g2.setFont(u8g2_font_helvB12_tf);
  u8g2.setCursor(0, 12);
  u8g2.print("Windguru TX20");
  u8g2.sendBuffer();
  #endif
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long wifi_timeout = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifi_timeout >= 30000) {
      ESP.reset();
      delay(5000);
    }
  }
  Serial.println("");
  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  #ifdef OLED
  u8g2.setCursor(0, 40);
  u8g2.print(WiFi.localIP());
  
  u8g2.sendBuffer();
  u8g2.clearBuffer();
  #endif  
  interval_count = millis(); 
}

boolean readTX20() {
    int bitcount=0;

    unsigned char chk;
    unsigned int sa,sb,sd,se;
    unsigned int sc,sf, pin;

    sa=sb=sd=se=0;
    sc=0;sf=0;
    String tx20RawDataS = "";

    for (bitcount=41; bitcount>0; bitcount--) {
      pin = (digitalRead(DATAPIN));
      if (!pin) {
        tx20RawDataS += "1";      
      } else {
        tx20RawDataS += "0";      
      }
      if ((bitcount==41-4) || (bitcount==41-8) || (bitcount==41-20)  || (bitcount==41-24)  || (bitcount==41-28)) {
        tx20RawDataS += " ";
      }      
      if (bitcount > 41-5){
        // start, inverted
        sa = (sa<<1)|(pin^1);
      } else
      if (bitcount > 41-5-4){
        // wind dir, inverted
        sb = sb>>1 | ((pin^1)<<3);
      } else
      if (bitcount > 41-5-4-12){
        // windspeed, inverted
        sc = sc>>1 | ((pin^1)<<11);
      } else
      if (bitcount > 41-5-4-12-4){
        // checksum, inverted
        sd = sd>>1 | ((pin^1)<<3);
      } else 
      if (bitcount > 41-5-4-12-4-4){
        // wind dir
        se = se>>1 | (pin<<3);
      } else {
        // windspeed
        sf = sf>>1 | (pin<<11);
      } 
          
      delayMicroseconds(1220);    
    }
    
    chk = ( sb + (sc&0xf) + ((sc>>4)&0xf) + ((sc>>8)&0xf) );chk&=0xf;
    
    delayMicroseconds(2000);  // just in case
    TX20IncomingData = false;  

    if (chk == sd) {          // if checksum seems to be ok
      quarter = dir[sb];
      dir_now  = float(sb)*22.5;
      wind_now = float(sc)/10;
  
      if (wind_now > wind_max) wind_max = wind_now;
      if (wind_min == 0) wind_min = wind_now;
      if (wind_now < wind_min) wind_min = wind_now;
      
      wind_num++;
      wind_sum += wind_now;
      wind_avg = wind_sum / wind_num;

      dir_num++;
      dir_sum += dir_now;
      wind_dir = dir_sum / dir_num;

      return true;
    } else {
      return false;      
    }
}

/*
 * LOOP
 */
void loop() {
  if (TX20IncomingData) {
    if (readTX20()) {
      Serial.print(quarter);
      Serial.print(" ");
      Serial.print(wind_dir);
      Serial.print("° ");
      Serial.print(wind_min);
      Serial.print("/");
      Serial.print(wind_avg);
  
      Serial.print("/");
      Serial.println(wind_max);

      #ifdef OLED
      u8g2.setFont(u8g2_font_inb16_mf);
      u8g2.setCursor(0, 16);
      u8g2.print(quarter);
      u8g2.print("  ");


      u8g2.setCursor(64, 16);
      u8g2.print(wind_dir);
      u8g2.print("° ");
      
      u8g2.setFont(u8g2_font_inb30_mf);
      u8g2.setCursor(0, 60);
      u8g2.print(wind_avg);
      u8g2.print("  ");      
      u8g2.sendBuffer();
      #endif
    }
  }

  if (millis() - interval_count >= interval ) {
    ntp.update();
    long epoch = ntp.getEpochTime();

    String url = "/upload/api.php?uid=";
    url += uid;
    url += "&salt=";
    url += epoch;
    url += "&hash=";
    url += md5(String(epoch)+String(uid)+String(apipass));
    url += "&wind_avg=";
    url += wind_avg * 1.943844;
    url += "&wind_min=";
    url += wind_min * 1.943844;
    url += "&wind_max=";
    url += wind_max * 1.943844;
    url += "&wind_direction=";
    url += wind_dir;

    #ifdef DEBUG
    Serial.println(url);
    #endif
    WiFiClient client;
    if (client.connect(host, 80)) {
      client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" + 
                 "Connection: close\r\n\r\n");
      delay(500);
    }

    bool ok = false;
    while(client.available()) {
      String line = client.readStringUntil('\n');   // read until the end of header and body 
      #ifdef DEBUG
      Serial.println(line);
      #endif
      if (line.startsWith("OK")) {                  // Server confirms that the request was "OK"
        ok = true;
        Serial.println("Server accepted the data");
      }
    }

    if (not ok) {
      ESP.reset();
      delay(5000);   
    }

    // reset the data used for averaging
    wind_num = 0;
    wind_sum = 0;
    wind_max = wind_avg;
    
    dir_num  = 0;
    dir_sum  = 0;
    
    interval_count = millis();
  }
}
