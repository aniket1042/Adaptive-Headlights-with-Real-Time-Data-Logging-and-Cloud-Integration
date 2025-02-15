#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Vamsy";
const char* password = "012345679";

// ThingsBoard server and device access token
const char* mqtt_server = "demo.thingsboard.io";
const char* deviceToken = "33f7ej6qmg74093q52iu";  // Replace with your actual device token

WiFiClient espClient;
PubSubClient client(espClient);

#define RX_PIN 16  // Define the RX pin for UART communication
#define TX_PIN 17  // Define the TX pin for UART communication

// Define variables to store the separated data
uint8_t SMA, SOL, IND, LDR_U, LDR_L;
uint16_t LDR;
// Set the custom MAC address in case your ESP32 is not regsitered with the acts network



void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Initialize UART2
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Initialize MQTT client
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  // Wait for serial communication to initialize
  delay(2000);
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.reconnect();
    delay(2000);
  }

  // Reconnect to MQTT if necessary
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop(); // Handle MQTT communication

  // Check if there are at least 5 bytes available to read (assuming 5 bytes from STM32)
  if (Serial2.available() >= 5) {
    // Read the 5 bytes from UART2
    SOL = Serial2.read();  // Read the first byte
    SMA = Serial2.read();  // Read the second byte
    IND = Serial2.read();  // Read the third byte
    LDR_U = Serial2.read();  // Read the fourth byte
    LDR_L = Serial2.read();  // Read the fifth byte

    // Combine the upper nibble and lower nibble for LDR
    LDR = ((LDR_U & 0x00FF) << 8) | (LDR_L & 0x00FF);

    // Print the received data to the Serial Monitor for debugging
    Serial.print("Received SMA: ");
    Serial.println(SMA);
    Serial.print("Received SOL: ");
    Serial.println(SOL);
    Serial.print("Received IND: ");
    Serial.println(IND);
    Serial.print("Received LDR: ");
    Serial.println(LDR);

    // Create JSON payload
    String payload = "{\"SMA\": " + String(SMA) + ", \"SOL\": " + String(SOL) + ", \"IND\": " + String(IND) + ", \"LDR\": " + String(LDR) + "}";

    
    // Publish the data to ThingsBoard
    if(client.publish("v1/devices/me/telemetry", payload.c_str())) {
        Serial.println("Data sent to ThingsBoard!");
        LDR=0;
    } else {
        Serial.println("Failed to send data to ThingsBoard.");
    }

    delay(2000); // Wait for 2 seconds before sending the next message
  }
}

// MQTT reconnect function
void reconnectMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect("ESP32_1", deviceToken, "")) {  // Unique ID for client
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

// MQTT callback function (optional, can be used for receiving messages)
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages (optional)
}
