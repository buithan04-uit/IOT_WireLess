#include "wifiConfig.h"
#include <HTTPClient.h>
#define BLYNK_TEMPLATE_ID "TMPL6GpkOr8sq"
#define BLYNK_TEMPLATE_NAME "esp32"
#define BLYNK_AUTH_TOKEN "Al3j40twtsPG10LOiEaNjhT6tb7IdjUL"
#include <BlynkSimpleEsp32.h>
// Dữ liệu giả lập
float temp = 12.5;             // °C
float humi = 45.4;             // %
int PM_1 = 11;        // µg/m³
int PM_2 = 12; 
int PM_3 = 123; 
double Lat = 10.762622;    // Vĩ độ
double Long = 106.660172;   // Kinh độ
// UART buffer
String uartBuffer = "";
bool blynkConnect=0;
const char* serverURL = "https://script.google.com/macros/s/AKfycbw1ZK-isZ4k76ybNOCAmKc4EbGU_n5u0iPZ1sIv6rfuewI9ARVxmHRMLmL69XTK2240/exec";

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V1,V2,V3,V4,V5,V6);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  wifiConfig.begin();

  Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 80);
}
void loop() {
  wifiConfig.run();

  if(WiFi.status()==WL_CONNECTED){
    if(blynkConnect==0){
      Serial.println("Connecting to blynk cloud...!");
      if(Blynk.connect(5000)){ 
        Serial.println("Connected to blynk cloud!");
        blynkConnect=1;
      }else{
        Serial.println("Connection failed. Try again later.");
      }
    }
    if (!Blynk.connected()) blynkConnect=0;
    Blynk.run();

    // Nhận dữ liệu UART
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n') {
        parseUARTData(uartBuffer);
        Serial.println(uartBuffer);
        uartBuffer = "";
      } else {
        uartBuffer += c;
      }
    }
  }
}

// Tách chuỗi UART và cập nhật biến
void parseUARTData(String data) {
  // Ví dụ chuỗi: T=45;H=70;PM1.0=120;PM2.5=120;PM10.0=120;LAT=10.762622;LON=106.660172
  int tIndex = data.indexOf("TEMP=");
  int hIndex = data.indexOf("HUM=");
  int pmIndex_1 = data.indexOf("PM1.0=");
  int pmIndex_2 = data.indexOf("PM2.5=");
  int pmIndex_3 = data.indexOf("PM10.0=");
  int latIndex = data.indexOf("LAT=");
  int lonIndex = data.indexOf("LON=");

  if (tIndex != -1) temp = data.substring(tIndex + 5, data.indexOf(';', tIndex)).toFloat();
  if (hIndex != -1) humi = data.substring(hIndex + 4, data.indexOf(';', hIndex)).toFloat();
  if (pmIndex_1 != -1) PM_1 = data.substring(pmIndex_1 + 6, data.indexOf(';', pmIndex_1)).toInt();
  if (pmIndex_2 != -1) PM_2 = data.substring(pmIndex_2 + 6, data.indexOf(';', pmIndex_2)).toInt();
  if (pmIndex_3 != -1) PM_3 = data.substring(pmIndex_3 + 7, data.indexOf(';', pmIndex_3)).toInt();
  if (latIndex != -1) Lat = data.substring(latIndex + 4, data.indexOf(';', latIndex)).toFloat();
  if (lonIndex != -1) Long = data.substring(lonIndex + 4).toFloat();

  // Gửi dữ liệu lên các datastream khi đã kết nối
  if (blynkConnect) {
    Blynk.virtualWrite(V0, temp);
    Blynk.virtualWrite(V1, humi);
    Blynk.virtualWrite(V2, PM_1);
    Blynk.virtualWrite(V3, PM_2);
    Blynk.virtualWrite(V4, PM_3);
    Blynk.virtualWrite(V5, Lat);
    Blynk.virtualWrite(V6, Long);
  }
  sendToGoogleSheet(temp, humi, PM_1 , PM_2 , PM_3 , Lat, Long);

  Serial.println(">> Parsed from UART:");
  Serial.printf("Temp: %.1f, Humi: %.1f, PM1.0: %d, PM2.5: %d, PM10.10: %d, Lat: %.6f, Lon: %.6f\n", temp, humi, PM_1 , PM_2 , PM_3 , Lat, Long);
}

void sendToGoogleSheet(float temp, float humi, int pm_1 , int pm_2 , int pm_3, float lat, float lon) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

  String jsonData = String("{\"temp\":") + String(temp, 1) +
                    ",\"humi\":" + String(humi, 1) +
                    ",\"pm_1\":" + String(pm_1) +
                    ",\"pm_2\":" + String(pm_2) +
                    ",\"pm_3\":" + String(pm_3) +
                    ",\"lat\":" + String(lat, 6) +
                    ",\"lon\":" + String(lon, 6) + "}";

    int httpResponseCode = http.POST(jsonData);
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Server reply: " + response);

    http.end();
  }
}
