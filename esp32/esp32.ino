#include <WiFi.h>
#include <ESPmDNS.h>

// ---- Configuration ----
const char* WIFI_SSID = "ssid";
const char* WIFI_PASS = "pass";
const char* HOSTNAME  = "nxpcup-bridge";

#define UART_RX 25   // ESP32 RX  <- MCXN947 TX (P2_3)
#define UART_TX 26   // ESP32 TX  -> MCXN947 RX (P2_4)
#define UART_BAUD 115200

WiFiServer tcpServer(3333);
WiFiClient tcpClient;

void setup() {
  Serial.begin(115200);              // USB serial, for local debugging only
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 30) {  // 15s timeout
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed. Starting AP mode...");
    WiFi.softAP("nxpcup-bridge", "12345678");
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  if (MDNS.begin(HOSTNAME)) {
    Serial.println("mDNS started: nxpcup-bridge.local");
    MDNS.addService("nxpcup", "tcp", 3333);
  }

  tcpServer.begin();
  tcpServer.setNoDelay(true);
  Serial.println("TCP command bridge ready on port 3333.");
}

void loop() {
  // Listen for incoming TCP clients (e.g., a python script or Netcat terminal)
  WiFiClient client = tcpServer.available();
  
  if (client) {
    Serial.println("TCP Client connected!");
    
    // Keep looping as long as the client remains connected
    while (client.connected()) {
      
      // 1. WiFi -> NXP: Read TCP data and transmit via UART
      while (client.available() > 0) {
        char c = client.read();
        Serial2.write(c); // Sends to NXP MCX N947
        
        // Optional: Echo to USB serial to monitor traffic
        Serial.write(c); 
      }
      
      // 2. NXP -> WiFi: Read UART data and send back over TCP
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        client.write(c);
      }
    }
    
    client.stop();
    Serial.println("TCP Client disconnected.");
  }
}