#include <Arduino.h>


int trigPin = 5;
int echoPin = 18;

void setup() {
 
  Serial.begin(115200);
  
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  
  // digitalWrite(trigPin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPin, LOW);

  
  // long duration = pulseIn(echoPin, HIGH);
  // int distance = duration * 0.034 / 2;
  

  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");

  // delay(1000);

  Serial.println("Hello, World! ");
  

}