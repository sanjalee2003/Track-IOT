#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// Wi-Fi credentials
const char* ssid = "iPhone SE 3";
const char* password = "senithi1234";

//Backend API endpoint where the ESP32 POSTs sensor data
const char* serverUrl = "http://172.20.10.4:3001/api/data";

// LED Pins
#define RED_LED     33
#define BLUE_LED    26
#define GREEN_LED   25
#define ORANGE_LED  32

// Ultrasonic Sensor Pins
#define TRIG_PIN    12   // MTDI (strap) — must be LOW at boot
#define ECHO_PIN    13

// Buzzer Pin
#define BUZZER_PIN  27

unsigned long lastSent = 0;
const long sendInterval = 200; // 

// Thresholds 
const float movementThreshold = 0.05;//(meters per second squared)
const float hitThreshold = 1.75;       
const float accelThreshold = 0.75;
const int obstacleDistance = 25;

Adafruit_MPU6050 mpu;
float prevAccel = 0;

// Collision detection with persistence
bool collisionDetected = false;
unsigned long collisionStartTime = 0;
const unsigned long collisionDuration = 300; // Keep collision flag true for 300ms
bool collisionSent = false; // Track if this collision was already sent

// Acceleration LED latch
unsigned long accelLEDTimer = 0;
const unsigned long accelLEDDuration = 500;
bool accelLEDActive = false;

// WiFi reconnection tracking
unsigned long lastWiFiCheck = 0;
const long wifiCheckInterval = 5000; // Check WiFi every 5 seconds

void forceTrigLowAtBoot() {
  pinMode(TRIG_PIN, INPUT_PULLDOWN);
  delay(20);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  forceTrigLowAtBoot();

  // LED setup
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);

  // Buzzer setup
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Ultrasonic setup
  pinMode(ECHO_PIN, INPUT);

  // MPU setup
  Wire.begin();//starts I2c Connection
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);//measure up to ±8 times the force of gravity 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);//measure rotation upto  to ±500 degrees per second
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);//smooths out noisy signals from the sensor.

  connectWiFi();
  Serial.println("Setup complete.");
}

void loop() {

  // Check WiFi connection periodically
  if (millis() - lastWiFiCheck >= wifiCheckInterval) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Reconnecting...");
      connectWiFi();
    }
  }

  // Read sensors
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Calculate acceleration magnitude
  float currentAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);//dot prodict vector
  float accelChange = fabs(currentAccel - prevAccel);//How much did it change since the last moment
  prevAccel = currentAccel;

 
 //Detecting a new collision
  if (accelChange > hitThreshold) {
    if (!collisionDetected) {
      // New collision detected
      collisionDetected = true;
      collisionStartTime = millis();
      collisionSent = false; // Reset the sent flag for new collision
      Serial.println("NEW COLLISION DETECTED! AccelChange: " + String(accelChange));
    }
  }

  // Clearing the collision after 500ms if it has not 
  if (collisionDetected && (millis() - collisionStartTime >= collisionDuration)) {
    collisionDetected = false;
    Serial.println("Collision flag cleared");
  }

  int distance = getDistance();

  // Reset LEDs
  resetLEDs();

  // Respond to the collission 
  if (collisionDetected) {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Handle obstacle detection
  if (distance > 0 && distance < obstacleDistance) {
    digitalWrite(ORANGE_LED, HIGH);
  }

  // Handle acceleration detection
  if (accelChange > accelThreshold) {
    accelLEDActive = true;
    accelLEDTimer = millis();
  }

  // Keep acceleration LED on for duration
  if (accelLEDActive) {
    digitalWrite(BLUE_LED, HIGH);
    if (millis() - accelLEDTimer >= accelLEDDuration) {
      accelLEDActive = false;
    }
  }

  // Handle movement detection
  if (accelChange > movementThreshold && !accelLEDActive) {
    digitalWrite(GREEN_LED, HIGH);
  }

  // Send data with collision priority
  if (millis() - lastSent >= sendInterval && WiFi.status() == WL_CONNECTED) {
    lastSent = millis();
    
    // Force immediate send if collision detected but not yet sent
    if (collisionDetected && !collisionSent) {
      Serial.println("SENDING COLLISION DATA IMMEDIATELY!");
      collisionSent = true; // Mark as sent to avoid duplicates
    }
    
    sendData(accelX, accelY, accelZ, collisionDetected, distance);
  }
  
  // EMERGENCY: If collision detected but WiFi down, try to reconnect immediately
  if (collisionDetected && !collisionSent && WiFi.status() != WL_CONNECTED) {
    Serial.println("COLLISION DETECTED BUT WIFI DOWN - EMERGENCY RECONNECT!");
    connectWiFi();
  }

  delay(50);
}

void resetLEDs() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.034 / 2;

  if (distance <= 0 || distance > 400) return -1;
  return distance;
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

void sendData(float ax, float ay, float az, bool collision, int distance) {
  //Check if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot send data - WiFi not connected");
    return;
  }
  //prepares for communication with the server
  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000); // 3 second timeout to check whether the server answers back


  String json = "{";
  json += "\"accelX\":" + String(ax, 2) + ",";
  json += "\"accelY\":" + String(ay, 2) + ",";
  json += "\"accelZ\":" + String(az, 2) + ",";
  json += "\"collision\":" + String(collision ? "true" : "false") + ",";
  json += "\"distance\":" + String(distance);
  json += "}";

  Serial.println("Sending: " + json);
  
  //send the json data to the server 
  int responseCode = http.POST(json);
  
  //checks whether the data was sent successfully using the response code
  if (responseCode > 0) {
    Serial.println("Response code: " + String(responseCode));
    if (collision) {
      Serial.println("✅ COLLISION DATA SENT SUCCESSFULLY!");
    }
  } else {
    Serial.println("❌ HTTP POST failed: " + String(responseCode));
    if (collision) {
      Serial.println("❌ FAILED TO SEND COLLISION DATA!");
    }
  }
  
  http.end();
}
