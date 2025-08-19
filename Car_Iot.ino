#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TFT_eSPI.h>
#include <ArduinoJson.h>

// ================== SERVO SYSTEM ==================
const int servoPins[3]  = {18, 19, 21};
const int sensorPins[3] = {32, 34, 35};

const int homePos[3]   = {25, 25, 105};
const int targetPos[3] = {110, 115, 205};

const int triggerThreshold = 500;
const int stableTime = 100;
const int holdTime = 5000;
const int cooldownTime = 1000;

Servo servos[3];
unsigned long lastBelowThreshold[3] = {0,0,0};
bool servoTriggered[3] = {false,false,false};
unsigned long triggerTime[3] = {0,0,0};
unsigned long cooldownStart[3] = {0,0,0};

// ================== WIFI + TFT DISPLAY ==================
const char* ssid = "iPhone SE 3";
const char* password = "senithi1234";

const char* irUrl = "http://172.20.10.4:3001/api/ir";
const char* currentUrl = "http://172.20.10.4:3001/api/current";

const int irPin = 13;
bool lastIrState = LOW;
unsigned long lastUpdateTime = 0;
unsigned long lastIrTriggerTime = 0;
int lastDisplayedTime = -1;
String lastDisplayedName = "";
bool gameWasFinished = false;
unsigned long gameOverShownTime = 0;
bool statsShown = false;

TFT_eSPI tft = TFT_eSPI();

// ================== STEPPER - FIXED ==================
#define STEP_PIN 12
#define DIR_PIN  14
const int stepsFor85Degrees = 47;
const int stepDelay = 1500;
const int stepperIrThreshold = 1300;  // Consistent threshold value
bool stepperTriggered = false;
bool stepperInMotion = false;
unsigned long stepperStartTime = 0;
unsigned long lastTriggerTime = 0;
const unsigned long preventRetriggerTime = 8000;

// ================== DC MOTOR ==================
#define IN1 25
#define IN2 33
#define ENA 26
const int pwmChannel = 0;
const int freq = 1000;
const int resolution = 8;
int motorSpeed = 255;
int onDuration = 500;
int offDuration = 3000;
unsigned long lastMotorToggle = 0;
bool motorOn = false;
bool motorInitialized = false;

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  // Servos
  for (int i = 0; i < 3; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(homePos[i]);
  }

  // TFT + WiFi
  pinMode(irPin, INPUT_PULLUP);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 10);
  tft.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.println("WiFi Connected.");

  // Stepper - FIXED
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  Serial.println("Stepper motor initialized");
  Serial.printf("Steps for 85 degrees: %d\n", stepsFor85Degrees);
  Serial.printf("IR threshold for stepper: >%d\n", stepperIrThreshold);

  // DC motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  if (!ledcAttach(ENA, freq, resolution)) {
    Serial.println("Failed to attach LEDC to pin");
  } else {
    Serial.println("LEDC attached successfully");
    motorInitialized = true;
  }
  
  ledcWrite(ENA, motorSpeed);
  motorOn = true;
  lastMotorToggle = millis();
  
  Serial.printf("DC Motor initialized: Speed=%d, On=%dms, Off=%dms\n", 
                motorSpeed, onDuration, offDuration);
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();

  handleServos(now);
  handleWiFiTFT(now);
  handleStepper(now);
  handleDCMotor(now);

  delay(20);
}

// ================== FUNCTIONS ==================
void handleServos(unsigned long now) {
  for (int i = 0; i < 3; i++) {
    int sensorValue = readAveraged(sensorPins[i], 5);
    if (!servoTriggered[i]) {
      if (now - cooldownStart[i] < cooldownTime) continue;
      if (sensorValue > triggerThreshold) {
        if (lastBelowThreshold[i] == 0) lastBelowThreshold[i] = now;
        else if (now - lastBelowThreshold[i] >= stableTime) {
          servos[i].write(targetPos[i]);
          servoTriggered[i] = true;
          triggerTime[i] = now;
          Serial.printf("Servo %d TRIGGERED\n", i+1);
        }
      } else lastBelowThreshold[i] = 0;
    } else {
      if (now - triggerTime[i] >= holdTime) {
        servos[i].write(homePos[i]);
        servoTriggered[i] = false;
        cooldownStart[i] = now;
        lastBelowThreshold[i] = 0;
        Serial.printf("Servo %d RESET\n", i+1);
      }
    }
  }
}

void handleWiFiTFT(unsigned long now) {
  bool currentIrState = digitalRead(irPin);
  if (lastIrState == LOW && currentIrState == HIGH && WiFi.status() == WL_CONNECTED) {
    if (now - lastIrTriggerTime >= 10000) {
      lastIrTriggerTime = now;
      HTTPClient http;
      http.begin(irUrl);
      http.addHeader("Content-Type", "application/json");
      int code = http.POST("");
      Serial.print("POST /api/ir -> ");
      Serial.println(code);
      http.end();
    } else {
      Serial.println("IR trigger ignored (cooldown active)");
    }
  }
  lastIrState = currentIrState;

  if (now - lastUpdateTime > 300 && WiFi.status() == WL_CONNECTED) {
    lastUpdateTime = now;
    HTTPClient http;
    http.begin(currentUrl);
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      StaticJsonDocument<512> doc;
      if (!deserializeJson(doc, payload)) {
        int seconds = doc["time"] | 0;
        bool started = doc["started"] | false;
        bool finished = doc["finished"] | false;
        int laps = doc["laps"] | 0;
        int collisions = doc["collisions"] | 0;
        int score = doc["score"] | 0;
        const char* name = doc["name"] | "";

        if (!started) {
          tft.fillScreen(TFT_BLACK);
          tft.setTextFont(2);
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setCursor(5, 30);
          tft.println("Game Starting...");
          lastDisplayedTime = -1; gameWasFinished = false; statsShown = false;
        } else if (seconds != lastDisplayedTime || String(name) != lastDisplayedName) {
          lastDisplayedTime = seconds;
          lastDisplayedName = String(name);
          tft.fillScreen(TFT_BLACK);

          tft.setTextFont(2);
          tft.setTextColor(TFT_CYAN, TFT_BLACK);
          tft.setCursor(10, 10);
          tft.print("Player: ");
          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.println(name);

          tft.setTextFont(6);
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setCursor(10, 60);
          tft.printf("%d s", seconds);
        }

        if (finished && !gameWasFinished) {
          tft.setTextFont(4);
          tft.setTextColor(TFT_BLUE, TFT_BLACK);
          tft.setCursor(10, 120);
          tft.println("Game Over!");
          gameOverShownTime = millis();
          gameWasFinished = true; statsShown = false;
        }

        if (gameWasFinished && !statsShown && millis() - gameOverShownTime > 4000) {
          tft.fillScreen(TFT_BLACK);

          tft.setTextFont(2);
          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.setCursor(10, 10); tft.print("Player: ");
          tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.println(name);

          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.setCursor(10, 30); tft.print("Time: ");
          tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.printf("%d s\n", seconds);

          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.setCursor(10, 50); tft.print("Laps: ");
          tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.println(laps);

          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.setCursor(10, 70); tft.print("Collisions: ");
          tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.println(collisions);

          tft.setTextFont(4);
          tft.setTextColor(TFT_BLUE, TFT_BLACK);
          tft.setCursor(10, 100); tft.print("Score: ");
          tft.println(score);

          statsShown = true;
        }
      }
    }
    http.end();
  }
}

// ================== UPDATED STEPPER FUNCTION ==================
void handleStepper(unsigned long now) {
  // Handle ongoing stepper motion (return to home after 35 seconds)
  if (stepperInMotion) {
    if (now - stepperStartTime >= 35000) { // 35 seconds holding
      Serial.println("Returning stepper to home position...");
      digitalWrite(DIR_PIN, LOW);  // RIGHT direction to return home
      rotateStepper(stepsFor85Degrees + 5);  // Extra 5 steps for full return
      stepperInMotion = false;
      stepperTriggered = false;
      lastTriggerTime = now;
      Serial.println("Stepper returned to home position");
      Serial.println("Waiting for next trigger (8 second cooldown)...");
    }
    return; // Don't check for new triggers while in motion
  }

  // Read IR sensor on GPIO 34
  int irValue = analogRead(34);

  // Print IR value every 2 seconds
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 2000) {
    Serial.printf("IR Value: %d (Trigger threshold: >%d)\n", irValue, stepperIrThreshold);
    lastPrint = now;
  }

  bool irTriggered = (irValue > stepperIrThreshold);
  bool cooldownExpired = (now - lastTriggerTime >= preventRetriggerTime);

  // Stepper pending 10-second delay
  static bool stepperPending = false;
  static unsigned long stepperPendingTime = 0;

  if (irTriggered && !stepperTriggered && cooldownExpired && !stepperPending) {
    stepperPending = true;
    stepperPendingTime = now;
    Serial.println("Stepper IR detected on pin 34, waiting 10 seconds...");
  }

  if (stepperPending && now - stepperPendingTime >= 10000) { // 10-second delay
    Serial.println("10 seconds passed, moving stepper 85 degrees LEFT...");
    digitalWrite(DIR_PIN, HIGH); // LEFT rotation
    rotateStepper(stepsFor85Degrees);
    stepperTriggered = true;
    stepperInMotion = true;
    stepperStartTime = now;
    stepperPending = false;
    Serial.println("Stepper moved and holding position...");
  }
}

// ================== DC MOTOR ==================
void handleDCMotor(unsigned long now) {
  if (!motorInitialized) return;
  
  if (now - lastMotorToggle >= (motorOn ? onDuration : offDuration)) {
    motorOn = !motorOn;
    lastMotorToggle = now;
    
    if (motorOn) {
      ledcWrite(ENA, motorSpeed);
      Serial.printf("Motor ON: Speed=%d for %dms\n", motorSpeed, onDuration);
    } else {
      ledcWrite(ENA, 0);
      Serial.printf("Motor OFF for %dms\n", offDuration);
    }
  }
}

int readAveraged(int pin, int samples) {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delayMicroseconds(200);
  }
  return total / samples;
}

void rotateStepper(int steps) {
  Serial.printf("Executing %d steps...\n", steps);
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
    
    if ((i + 1) % 10 == 0) {
      Serial.printf("Step %d/%d\n", i + 1, steps);
    }
  }
  
  Serial.println("Step sequence completed");
}
