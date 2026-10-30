// // #define CAM_RX 16 
// // #define CAM_TX 17
// // bool camFlagVisible = false;
// // float camError = 0.0f;
// // String camColor = "NONE";
// // unsigned long lastCamPacketTime = 0;
// // int trig = 5;
// // int echo = 18;
// // int in1R = 26;
// // int in2R = 14;
// // int enA = 33;
// // int in3L = 27;
// // int in4L = 25;
// // int enB = 32;
// // int speedspin = 120;
// // int deadband = 15;
// // float setpoint = 50.0;
// // float distancedeadband = 1.0;
// // int maxspeed = 200;
// // int minspeed = 120;
// // float kp = 2.0;
// // float ki = 0.1;
// // float kd = 1.5;
// // float integral = 0.0;
// // float lasterror = 0.0;
// // unsigned long lastTime = 0;
// // enum carstate {
// //   SEARCH,
// //   ALIGN,
// //   APPROACH,
// //   DONE
// // };
// // carstate state = SEARCH;
// // void moveforward(int speed){
// //   digitalWrite(in1R, HIGH);
// //   digitalWrite(in2R, LOW);
// //   analogWrite(enA, speed);
// //   digitalWrite(in3L, HIGH);
// //   digitalWrite(in4L, LOW);
// //   analogWrite(enB, speed);
// // }
// // void backward(int speed){
// //   digitalWrite(in1R, LOW);
// //   digitalWrite(in2R, HIGH);
// //   analogWrite(enA, speed);
// //   digitalWrite(in3L, LOW);
// //   digitalWrite(in4L, HIGH);
// //   analogWrite(enB, speed);
// // }
// // void stopmotor(){
// //   digitalWrite(in1R, LOW);
// //   digitalWrite(in2R, LOW);
// //   analogWrite(enA, 0);
// //   digitalWrite(in3L, LOW);
// //   digitalWrite(in4L, LOW);
// //   analogWrite(enB, 0);
// // }
// // void spinright(int speedspin){
// //   digitalWrite(in1R, LOW);
// //   digitalWrite(in2R, HIGH);
// //   analogWrite(enA, speedspin);
// //   digitalWrite(in3L, HIGH);
// //   digitalWrite(in4L, LOW);
// //   analogWrite(enB, speedspin);
// // }
// // void spinleft(int speedspin){
// //   digitalWrite(in1R, HIGH);
// //   digitalWrite(in2R, LOW);
// //   analogWrite(enA, speedspin);
// //   digitalWrite(in3L, LOW);
// //   digitalWrite(in4L, HIGH);
// //   analogWrite(enB, speedspin);
// // }
// // float readDistance(){
// //   digitalWrite(trig, LOW);
// //   delayMicroseconds(2);
// //   digitalWrite(trig, HIGH);
// //   delayMicroseconds(10);
// //   digitalWrite(trig, LOW);
// //   long duration = pulseIn(echo, HIGH, 30000);
// //   if(duration == 0) return -1;      // مفيش صدى
// //   return duration * 0.034 / 2;
// // }
// // void readCameraSerial() {
// //   while (Serial2.available() > 0) {
// //     String line = Serial2.readStringUntil('\n'); 
// //     line.trim();                                 
// //     if (line.length() < 3) continue;            
// //     int firstComma = line.indexOf(',');
// //     int secondComma = line.indexOf(',', firstComma + 1);
// //     if (firstComma != -1 && secondComma != -1) {
// //       int vis = line.substring(0, firstComma).toInt();                   
// //       float err = line.substring(firstComma + 1, secondComma).toFloat(); 
// //       String col = line.substring(secondComma + 1);
// //       col.trim();
// //       camFlagVisible = (vis == 1);
// //       camError = err;
// //       camColor = col;
// //       lastCamPacketTime = millis();
// //     }
// //   }
// //   if (millis() - lastCamPacketTime > 1000) {
// //     camFlagVisible = false;
// //   }
// // }
// // void setup() {
// //   Serial.begin(115200);
// //   Serial2.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);
// //   Serial2.setTimeout(20);
// //   pinMode(trig, OUTPUT);
// //   pinMode(echo, INPUT);
// //   pinMode(in1R, OUTPUT);
// //   pinMode(in2R, OUTPUT);
// //   pinMode(enA, OUTPUT);
// //   pinMode(in3L, OUTPUT);
// //   pinMode(in4L, OUTPUT);
// //   pinMode(enB, OUTPUT);
// //   stopmotor();
// //   lastTime = millis();
// // }
// // void loop() {
// //   readCameraSerial();
// //   if(state == SEARCH){
// //     if(camFlagVisible){
// //       stopmotor();
// //       state = ALIGN;
// //       return;
// //     }
// //     spinright(speedspin);
// //     delay(100);
// //     stopmotor();
// //     delay(500);
// //     readCameraSerial();           
// //     if(camFlagVisible){
// //       state = ALIGN;
// //     }
// //   }
// //   else if(state == ALIGN){
// //     if(!camFlagVisible){
// //       stopmotor();
// //       state = SEARCH;
// //     }
// //     else{
// //       float error = camError;     
// //       if(abs(error) <= deadband){
// //         stopmotor();
// //         integral = 0.0;
// //         float d = readDistance();
// //         if(d < 0){                
// //           return;
// //         }
// //         lasterror = d - setpoint;
// //         lastTime = millis();
// //         state = APPROACH;
// //       }
// //       else if(error < 0){
// //         spinleft(speedspin);
// //         delay(100);
// //         stopmotor();
// //         delay(500);
// //         readCameraSerial();
// //       }
// //       else{
// //         spinright(speedspin);
// //         delay(100);
// //         stopmotor();
// //         delay(500);
// //         readCameraSerial();
// //       }
// //     }
// //   }
// //   else if(state == APPROACH){
// //     float distance = readDistance();
// //     if(distance < 0){             
// //       delay(10);
// //       return;
// //     }
// //     float error = distance - setpoint; 
// //     if(abs(error) <= distancedeadband){
// //       stopmotor();
// //       Serial.print("Reached. distance: ");
// //       Serial.println(distance);
// //       state = DONE;
// //       return;
// //     }
// //     unsigned long now = millis();
// //     float dt = (now - lastTime) / 1000.0;
// //     if(dt <= 0){
// //       delay(10);
// //       return;
// //     }
// //     lastTime = now;
// //     integral += error * dt;
// //     integral = constrain(integral, -100, 100);
// //     float derivative = (error - lasterror) / dt;
// //     lasterror = error;
// //     float output = kp*error + ki*integral + kd*derivative;
// //     int speed = constrain((int)abs(output), minspeed, maxspeed);
// //     if(output > 0){
// //       moveforward(speed);     
// //     }
// //     else{
// //       backward(speed);
// //     }
// //     Serial.print("distance: ");
// //     Serial.print(distance);
// //     Serial.print(" output: ");
// //     Serial.println(output);
// //     delay(50);
// //   }
// //   else if(state == DONE){
// //     stopmotor();
// //   }
// // }









#include <Arduino.h>

// Ultrasonic Sensor Pins
int trig = 18;
int echo = 5;

// Motor Driver Pins
int in1R = 26;
int in2R = 14;
int enA = 33;
int in3L = 27;
int in4L = 25;
int enB = 32;

void moveforward(int speedl, int speedr) {
  // Right Motor (enA)
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  analogWrite(enA, speedr);

  // Left Motor (enB)
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, HIGH);
  analogWrite(enB, speedl);
}

void stopmotor() {
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);
  analogWrite(enA, 0);
  digitalWrite(in3L, LOW);
  digitalWrite(in4L, LOW);
  analogWrite(enB, 0);
}

float readDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return 400.0;
  return duration * 0.034 / 2.0;
}

void setup() {
  Serial.begin(115200);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in3L, OUTPUT);
  pinMode(in4L, OUTPUT);
  pinMode(enB, OUTPUT);
}

void loop() {
  float distance = readDistance();
  Serial.printf("Distance: %.2f cm\n", distance); 

  if (distance <= 3.0) {
    // stopmotor();
  } else {
    moveforward(100, 100);
  }

  delay(50); 
}