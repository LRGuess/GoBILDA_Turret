#include <Servo.h>
#include <AccessUltrasonic.h>

#define LASER_PIN 53
#define ULTRA_TRIG_PIN 61
#define ULTRA_ECHO_PIN 62

Servo rotor_ESC;
Servo feed_ESC;
Servo pan_ESC;
Servo tilt_ESC;

int rotorPower = 0;
int prevRotorPower = 0; 

void setup() {
  
  rotor_ESC.attach(5);
  feed_ESC.attach(4);

  Serial.begin(9600);

  pinMode(laserPin, OUTPUT);  
}

void loop() {
  
}

void idle(){
  digitalWrite(laserPin, LOW)
}

void searching(){

}

void fire() {
  rotor_ESC.writeMicroseconds(givePowerInPWM(rotorPower));
  feed_ESC.writeMicroseconds(givePowerInPWM(rotorPower) - 10);
}

int givePowerInPWM(int power) {
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); 
  return signal_output;
}
