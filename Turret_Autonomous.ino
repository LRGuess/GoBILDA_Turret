#include <Servo.h>
#include <AccessUltrasonic.ino>

#define laserPin 53

Servo rotor_ESC;
Servo feed_ESC;
Servo pan_ESC;
Servo tilt_ESC;

int rotorPower = 0;
int prevRotorPower = 0; 

int motionDetectorPin = ;

void setup() {
  
  rotor_ESC.attach(5);
  feed_ESC.attach(4);

  Serial.begin(9600);

  pinMode(laserPin, OUTPUT);  
}

void loop() {
  fireWaves();
  
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
