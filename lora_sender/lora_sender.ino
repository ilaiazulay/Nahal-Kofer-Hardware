#include <RadioLib.h>

// Define sensor pins
#define TRIG_PIN 25
#define ECHO_PIN 26  // Adjust if needed
#define FLOW_SENSOR_PIN 27 // Define pin for the water flow sensor
#define PH_SENSOR_PIN 34 // Define pin for the pH sensor (ADC1 channel)

#define PH_SENSOR_MIN 2.5 // Minimum expected pH value
#define PH_SENSOR_MAX 14.0 // Maximum expected pH value

// Define LoRa module pins
#define NSS_PIN 5    // SPI Chip Select
#define DIO0_PIN 32  // Dio0 pin used for interrupt
#define RST_PIN 14   // Reset pin

// Create a new instance of the LoRa module
SX1278 radio = new Module(NSS_PIN, DIO0_PIN, RST_PIN);

// Variables for the water flow sensor
volatile int flowPulseCount = 0;
float flowRate = 0.0;
unsigned long oldTime = 0;

// Calibration coefficients (example values, replace with actual coefficients)
float a = -2.5e-7;  // Replace with your calculated value
float b = -0.01;    // Replace with your calculated value
float c = 40;       // Replace with your calculated value

void IRAM_ATTR flowSensorISR() {
  flowPulseCount++;
}

void setup() {
  Serial.begin(9600);
  delay(1000); // Wait for Serial Monitor to open
  Serial.println("Starting setup...");

  // Initialize LoRa module at 433.0 MHz - check and adjust frequency as per your module's specs and region
  Serial.print(F("Initializing LoRa... "));
  int state = radio.begin(433.0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true); // Infinite loop on failure
  }

  // Set up sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);

  // Optionally, set the resolution for analogRead
  analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095)


  Serial.println("Setup complete");
}

void loop() {
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2000);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(2000);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration / 2) / 29.1;  // Calculate distance

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Calculate flow rate
  if ((millis() - oldTime) > 1000) { // Only process once per second
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
    flowRate = (flowPulseCount / 7.5); // Convert pulses to L/min (7.5 pulses per liter)
    oldTime = millis();
    flowPulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);

    Serial.print("Flow rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");

    // Read pH sensor value
    int phValueRaw = analogRead(PH_SENSOR_PIN);
    Serial.print("Raw pH Value: ");
    Serial.println(phValueRaw);

    float phVoltage = phValueRaw * (3.3 / 4095.0); // Convert raw reading to voltage
    Serial.print("pH Voltage: ");
    Serial.println(phVoltage);

    // Apply quadratic calibration to convert raw ADC value to pH
    float phValue = (a * phValueRaw * phValueRaw + b * phValueRaw + c) -1; // Calibrated pH value

    // Constrain the pH value within the expected range
    if (phValue < PH_SENSOR_MIN) {
      phValue = PH_SENSOR_MIN;
    } else if (phValue > PH_SENSOR_MAX) {
      phValue = PH_SENSOR_MAX;
    }

    Serial.print("pH value: ");
    Serial.println(phValue);

    // Send the distance, flow rate, and pH value over LoRa
    Serial.print(F("Sending packet... "));
    char buffer[32];  // Increase buffer size if needed
    sprintf(buffer, "%.2f,%.2f,%.2f", distance, flowRate, phValue);
    Serial.print("Packet content: ");
    Serial.println(buffer);

    int state = radio.transmit(buffer);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
  }

  delay(1000);
}
