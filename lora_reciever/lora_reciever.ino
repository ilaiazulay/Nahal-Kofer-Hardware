#include <RadioLib.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi and MQTT setup
const char* ssid = "Shenkar-New";
const char* password = "Shenkarwifi";
const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_topic_distance = "sensors/ultrasonic/distance";
const char* mqtt_topic_flow = "sensors/water/flow";
const char* mqtt_topic_ph = "sensors/water/ph";

WiFiClient espClient;
PubSubClient client(espClient);

// LoRa module pins
#define NSS_PIN 5    // SPI Chip Select
#define DIO0_PIN 32  // Dio0 pin used for interrupt
#define RST_PIN 14   // Reset pin

// Create a new instance of the LoRa module
SX1278 radio = new Module(NSS_PIN, DIO0_PIN, RST_PIN);

void setup() {
  Serial.begin(9600);
  delay(1000); // Wait for Serial Monitor to open
  Serial.println("Starting setup...");

  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Initialize LoRa module at 433.0 MHz (adjust this to your region and module specifications)
  Serial.println("Initializing LoRa...");
  int state = radio.begin(433.0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa receiver initialized successfully!");
  } else {
    Serial.print("LoRa initialization failed, error code: ");
    Serial.println(state);
    while (true); // Infinite loop on failure
  }
}

void setup_wifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
}

void loop() {
  String str;
  Serial.println("Waiting for LoRa packet...");
  int state = radio.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("Received data: ");
    Serial.println(str);

    // Parse the received string as comma-separated values
    int firstCommaIndex = str.indexOf(',');
    int secondCommaIndex = str.indexOf(',', firstCommaIndex + 1);

    if (firstCommaIndex == -1 || secondCommaIndex == -1) {
      Serial.println("Invalid data format");
      return;
    }

    float distance = str.substring(0, firstCommaIndex).toFloat();
    float flowRate = str.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
    float phValue = str.substring(secondCommaIndex + 1).toFloat();

    Serial.print("Extracted distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    Serial.print("Extracted flow rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
    Serial.print("Extracted pH value: ");
    Serial.println(phValue);

    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    char msg_distance[50];
    char msg_flow[50];
    char msg_ph[50];
    sprintf(msg_distance, "{\"distance\": %.2f}", distance);
    sprintf(msg_flow, "{\"flow_rate\": %.2f}", flowRate);
    sprintf(msg_ph, "{\"ph_value\": %.2f}", phValue);

    Serial.print("Publishing distance: ");
    Serial.println(msg_distance);
    client.publish(mqtt_topic_distance, msg_distance);
    
    Serial.print("Publishing flow rate: ");
    Serial.println(msg_flow);
    client.publish(mqtt_topic_flow, msg_flow);

    Serial.print("Publishing pH value: ");
    Serial.println(msg_ph);
    client.publish(mqtt_topic_ph, msg_ph);
  } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.print("Error receiving: code ");
    Serial.println(state);
  }

  delay(100);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32ClientReceiver")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
