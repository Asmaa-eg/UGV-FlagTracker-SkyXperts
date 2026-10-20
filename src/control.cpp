#include <Arduino.h>
// Safe ESP32 GPIO pins (Avoid GPIOs 6-11 as they are connected to internal SPI flash)
const int trig = 4;
const int echo = 18;

const int in1R = 25;
const int in2R = 26;
const int enA  = 32;

const int in3L = 27;
const int in4L = 14;
const int enB  = 33;

// UART2 pins for receiving data from ESP32-CAM
#define CAM_RX_PIN 16
#define CAM_TX_PIN 17

enum carstate {
  SEARCH,
  ALIGN,
  APPROACH,
  STOPPED
};
carstate state = SEARCH;

// Vision state variables updated via UART
bool flagVisible = false;
float flagError = 0.0f;
String flagColor = "NONE";
bool colorAnnounced = false;

// ================= MOTOR CONTROL =================
void moveforward(int speed) {
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  analogWrite(enA, speed);
  digitalWrite(in3L, HIGH);
  digitalWrite(in4L, LOW);
  analogWrite(enB, speed);
}

void backward(int speed) {
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  analogWrite(enA, speed);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, HIGH);
  analogWrite(enB, speed);
}

void stopmotor() {
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);
  analogWrite(enA, 0);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, LOW);
  analogWrite(enB, 0);
}

void Drive(int speedright, int speedleft) {
  speedright = constrain(speedright, 0, 255);
  speedleft  = constrain(speedleft, 0, 255);

  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  analogWrite(enA, speedright);
  digitalWrite(in3L, HIGH);
  digitalWrite(in4L, LOW);
  analogWrite(enB, speedleft);
}

void spinright(int speedspin) {
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  analogWrite(enA, speedspin);
  digitalWrite(in3L, HIGH);
  digitalWrite(in4L, LOW);
  analogWrite(enB, speedspin);
}

void spinleft(int speedspin) {
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  analogWrite(enA, speedspin);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, HIGH);
  analogWrite(enB, speedspin);
}

// ================= SENSORS & VISION =================
float readDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) {
    return -1;
  }
  return duration * 0.034 / 2;
}

// Reads the latest packet from ESP32-CAM over Serial2 (Format: "1,0.25,RED")
void readCamera() {
  while (Serial2.available() > 0) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    
    int firstComma = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
      int vis = line.substring(0, firstComma).toInt();
      flagVisible = (vis == 1);
      flagError = line.substring(firstComma + 1, secondComma).toFloat();
      flagColor = line.substring(secondComma + 1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Hardware Serial2 for receiving ESP32-CAM data on GPIO 16
  Serial2.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in3L, OUTPUT);
  pinMode(in4L, OUTPUT);
  pinMode(enB, OUTPUT);

  Serial.println("Control ESP32 Ready!");
}

void loop() {
  readCamera();

  // ================= 1. SEARCH STATE =================
  if (state == SEARCH) {
    if (flagVisible) {
      stopmotor();
      
      // Announce flag color once upon initial detection (trigger LED/speaker here)
      if (!colorAnnounced) {
        Serial.println("Detected Flag Color: "+ flagColor);
        colorAnnounced = true;
      }
      
      delay(200);
      state = ALIGN;
    } else {
      // Short spin followed by a pause to prevent camera motion blur
      spinright(110);
      delay(80);
      stopmotor();
      delay(250);
    }
  }

  // ================= 2. ALIGN STATE =================
  else if (state == ALIGN) {
    if (!flagVisible) {
      stopmotor();
      state = SEARCH;
    } 
    // Target is centered within the dead zone
    else if (abs(flagError) <= 0.12f) {
      stopmotor();
      delay(150);
      state = APPROACH;
    } 
    else {
      int turnSpeed = 100;
      if (flagError > 0) {
        spinright(turnSpeed); // Flag is to the right
      } else {
        spinleft(turnSpeed);  // Flag is to the left
      }
      delay(40);
      stopmotor();
      delay(100);
    }
  }

  // ================= 3. APPROACH STATE =================
  else if (state == APPROACH) {
    if (!flagVisible) {
      stopmotor();
      state = SEARCH;
      return;
    }

    float dist = readDistance();
    // Stop when within 20 cm of the flag
    if (dist > 0 && dist <= 20.0f) {
      stopmotor();
      state = STOPPED;
      Serial.println("Reached Target Flag!");
      return;
    }

    // Proportional steering while driving forward
    int baseSpeed = 140;
    int correction = flagError * 55;

    int leftSpeed  = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    Drive(rightSpeed, leftSpeed);
    delay(30);
  }

  // ================= 4. STOPPED STATE =================
  else if (state == STOPPED) {
    stopmotor();
  }
}