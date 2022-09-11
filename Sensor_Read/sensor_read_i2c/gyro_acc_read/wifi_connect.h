#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Localhost";
const char* password = "8976152032";
WiFiUDP Udp;

void wifi_connect(){
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  }

void send_udp_packet(char sensor_data[]){
  Udp.beginPacket("192.168.0.105", 4000);
  Udp.write(sensor_data);
  Udp.endPacket();
  }
