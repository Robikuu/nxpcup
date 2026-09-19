#include <Arduino.h>
#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"
#include "WiFi.h"
#include "esp_wifi.h"

// ---- Configuration ----
const char* WIFI_SSID = "Robotics315";
const char* WIFI_PASS = "helloworld";
const char* HOSTNAME  = "nxpcup-bridge";

#define ESPNOW_WIFI_CHANNEL 1

#define UART_RX 25   // ESP32 RX  <- MCXN947 TX (P2_3)
#define UART_TX 26   // ESP32 TX  -> MCXN947 RX (P2_4)
#define UART_BAUD 115200

const MacAddress peer_mac({0x3C, 0x8A, 0x1F, 0x0D, 0x7E, 0x7C});

ESP_NOW_Serial_Class NowSerial(peer_mac, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA);

void setup() {
  Serial.begin(UART_BAUD);  // USB, free for debug logging on this board
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  NowSerial.begin(UART_BAUD);
  Serial.println("Bridge ready — relaying ESP-NOW <-> UART2.");
}


void loop() {
  // ESP32 #1 -> NXP board
  while (NowSerial.available()) {
    int c = NowSerial.read();
    if (c < 0) break;
    Serial2.write((uint8_t)c);
    Serial.write((uint8_t)c);   // debug echo of the same byte
  }

  // NXP board -> ESP32 #1
  while (Serial2.available() && NowSerial.availableForWrite()) {
    if (NowSerial.write(Serial2.read()) <= 0) {
      Serial.println("ESP-NOW send failed");
      break;
    }
  }
  delay(1);
}