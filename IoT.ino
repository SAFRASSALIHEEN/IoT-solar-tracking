#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Servo.h>

// Replace with your network credentials
const char* ssid = "SMK";
const char* password = "qwertyuiop";
   
// Adafruit IO settings
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "safras456"
#define AIO_KEY         "aio_YqNh22KW9Br4fnWy1Du7SbuyDgaN"

// LDR pins
const int LDR_TOP = 36;       // VP
const int LDR_BOTTOM = 39;    // VN
const int LDR_LEFT = 34;      // D34
const int LDR_RIGHT = 35;     // D35

// Servo pins
const int SERVO_PAN = 26;  // D26
const int SERVO_TILT = 27; // D27

// Voltage sensor pin
const int voltageSensorPin = 33;  

// Servo objects
Servo servoPan;
Servo servoTilt;

// Current servo positions
int panPos = 0;
int tiltPos = 90;

// Threshold for movement
const int THRESHOLD = 25;  // Reduced threshold for increased sensitivity

// Step size for servo movement
const int STEP_SIZE = 5;  // Increased step size for quicker movement

// Delay between movements (milliseconds)
const int MOVEMENT_DELAY = 25;  // Reduced delay for faster response

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish panPosFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/panPos");
Adafruit_MQTT_Publish tiltPosFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tiltPos");
Adafruit_MQTT_Publish avgLdrFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/avgLdrValue");
Adafruit_MQTT_Publish voltageFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/voltage");

void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void connectToMQTT() {
  while (mqtt.connected() == false) {
    Serial.print("Connecting to MQTT... ");
    if (mqtt.connect()) {
      Serial.println("connected");
    } else {
      Serial.print("failed, status code =");
      Serial.print(mqtt.connectErrorString(mqtt.connect()));
      Serial.println(", retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();

  servoPan.attach(SERVO_PAN);
  servoTilt.attach(SERVO_TILT);

  // Set initial position
  servoPan.write(panPos);
  servoTilt.write(tiltPos);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  
  if (!mqtt.connected()) {
    connectToMQTT();
  }

  mqtt.processPackets(10000);
  mqtt.ping();

  trackSun();
  sendVoltageData();
  delay(MOVEMENT_DELAY);
}

void trackSun() {
  int top = analogRead(LDR_TOP);
  int bottom = analogRead(LDR_BOTTOM);
  int left = analogRead(LDR_LEFT);
  int right = analogRead(LDR_RIGHT);

  int avgTop = (top + (left + right)/2) / 2;
  int avgBottom = (bottom + (left + right)/2) / 2;
  int avgLeft = (left + (top + bottom)/2) / 2;
  int avgRight = (right + (top + bottom)/2) / 2;
  
  int avgLDR = (top + bottom + left + right) / 4;

  bool panMoved = false;
  bool tiltMoved = false;

  if (abs(avgTop - avgBottom) > THRESHOLD) {
    if (avgTop > avgBottom) {
      tiltPos -= STEP_SIZE;
    } else {
      tiltPos += STEP_SIZE;
    }
    tiltPos = constrain(tiltPos, 0, 120);
    tiltMoved = true;
  }

  if (abs(avgLeft - avgRight) > THRESHOLD) {
    if (avgLeft > avgRight) {
      panPos += STEP_SIZE;
    } else {
      panPos -= STEP_SIZE;
    }
    panPos = constrain(panPos, 0, 120);
    panMoved = true;
  }

  // Move both servos simultaneously if needed
  if (panMoved) servoPan.write(panPos);
  if (tiltMoved) servoTilt.write(tiltPos);

  // Publish data to Adafruit IO
  if (panMoved) panPosFeed.publish(panPos);
  if (tiltMoved) tiltPosFeed.publish(tiltPos);

  // Publish average LDR value to Adafruit IO
  avgLdrFeed.publish(avgLDR);

  // Print positions for debugging
  Serial.print("Pan: ");
  Serial.print(panPos);
  Serial.print(" Tilt: ");
  Serial.println(tiltPos);

  // Print LDR values for debugging
  Serial.print("T: ");
  Serial.print(top);
  Serial.print(" B: ");
  Serial.print(bottom);
  Serial.print(" L: ");
  Serial.print(left);
  Serial.print(" R: ");
  Serial.println(right);
  
  // Print average LDR value for debugging
  Serial.print("Average LDR: ");
  Serial.println(avgLDR);
}

void sendVoltageData() {
  int sensorValue = analogRead(voltageSensorPin);  // Read the analog value
  float voltage = sensorValue * (25 / 4095.0);  // Convert to voltage (ESP32 has 12-bit ADC)

  // Print voltage for debugging
  Serial.print("Voltage: ");
  Serial.println(voltage);

  // Publish voltage to Adafruit IO
  voltageFeed.publish(voltage);
}
