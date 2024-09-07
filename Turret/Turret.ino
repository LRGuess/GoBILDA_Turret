#include <Servo.h>
#include <Encoder.h>
// Include iBusBM Library
#include <IBusBM.h>

#define LASER_PIN 53
#define TRIG_PIN 47
#define ECHO_PIN 49
#define ROTOR_ESC_PIN 11
#define PAN_PIN 12
#define TILT_PIN 13
#define LDRIVE_PIN 9
#define RDRIVE_PIN 10
#define MOTION_SENSOR_PIN 45
#define IR_DETECTION_PIN 41
#define R_PIN 29
#define G_PIN 25
#define B_PIN 27
#define SMOOTHING_FACTOR 0.2  // Smoothing factor for exponential moving average (0 < factor < 1)
#define DEBOUNCE_THRESHOLD 5  // Threshold for considering changes significant

// Smoothed input values for the channels
int smoothedValues[5] = {0, 0, 0, 0, 0};  // Smoothed values for Ch1 to Ch5


Servo rotor_ESC;
Servo pan;
Servo tilt;
Servo LDrive;
Servo RDrive;

IBusBM ibus;

//State
// 0 = idle
// 1 = searching 
// 2 = firing
int state = 0;

bool manual = false;

//Motor variables
int rotorPower = 0;

//Ultrasonic Variables
float filterArray[20]; // array to store data samples from sensor
float distance; // store the distance from sensor

//Motion Detection Variables
int pinStateCurrent   = LOW; // current state of pin
int pinStatePrevious  = LOW; // previous state of pin

//IR Avoidance Variables
int IRlastState = HIGH; // previous state of pin
int IRcurrentState; // current state of pin

void setup() {
  
  rotor_ESC.attach(ROTOR_ESC_PIN);
  pan.attach(PAN_PIN);
  tilt.attach(TILT_PIN);
  LDrive.attach(LDRIVE_PIN);
  RDrive.attach(RDRIVE_PIN);

  Serial.begin(115200);
  ibus.begin(Serial1);

  setPinModes();
}

void loop() {
  IBusSerial();
  if (readSwitch(6, true)){
    IBusSerial();
    manual = true;
    manualControl();
  } else {
    manual = false;
  }


  if (motionDetected() && !manual){
    searching();
  }
  else {
    idle();
  }
}

//State and operation Methods
//--------------------------------------------------------------------------------------

void idle(){
  digitalWrite(LASER_PIN, LOW);

  ledColor(144, 253, 147);
}

void searching(){
  digitalWrite(LASER_PIN, HIGH);

  ledColor(255, 255, 119);
}

void fire() {
  digitalWrite(LASER_PIN, HIGH);
  rotor_ESC.writeMicroseconds(givePowerInPWM(rotorPower));
}

//Setup 
//---------------------------------------------------------------------------------------

void setPinModes(){
  pinMode(LASER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(IR_DETECTION_PIN, INPUT);
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
}

//Development Methods
//--------------------------------------------------------------------------------------

void SerialPlot(){
  Serial.println(distance);
  Serial.print(" ");
  Serial.print(motionDetected());
  Serial.print(" ");
  Serial.print(IRDetected());
}

void IBusSerial() {
  for (byte i = 0; i < 6; i++) {
    int value = readChannel(i, -100, 100, 0);
    Serial.print("Ch");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(" ");
  }
  Serial.print("Smoothed Values: ");
  for (byte i = 0; i < 5; i++) {
    Serial.print(smoothedValues[i]);
    Serial.print(" ");
  }
  Serial.print("Ch7: ");
  Serial.print(readSwitch(6, false));
  Serial.print("Ch8: ");
  Serial.print(readSwitch(7, false));
  Serial.print("Ch9: ");
  Serial.print(readSwitch(8, false));
  Serial.print("Ch10: ");
  Serial.print(readSwitch(9, false));
  Serial.println();
  delay(10);
}

//Manual Methods
//---------------------------------------------------------------------------------------

void manualControl() {
  // Read input from channels and smooth the values
  for (byte i = 0; i < 5; i++) {
    int rawValue = readChannel(i, -100, 100, 0);
    smoothedValues[i] = smoothInput(smoothedValues[i], rawValue);
  }

  // Update servos and ESCs with smoothed values
  tilt.writeMicroseconds(givePowerInPWM(smoothedValues[2]));  // Channel 2 for tilt
  pan.writeMicroseconds(givePositionInPWM(smoothedValues[3])); // Channel 3 for pan
  rotor_ESC.writeMicroseconds(givePowerInPWM(smoothedValues[4])); // Channel 4 for rotor
  LDrive.writeMicroseconds(givePowerInPWM(smoothedValues[0])); // Channel 1 for left drive
  RDrive.writeMicroseconds(givePowerInPWM(smoothedValues[1])); // Channel 5 for right drive
}

int smoothInput(int previousValue, int newValue) {
  // If the change is greater than the debounce threshold, apply smoothing
  if (abs(previousValue - newValue) > DEBOUNCE_THRESHOLD) {
    return previousValue + (newValue - previousValue) * SMOOTHING_FACTOR;
  }
  return previousValue;  // If change is small, retain previous value
}

int rotorPWM(int speed){
  speed = constrain(speed, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(speed, -100, 100, signal_min, signal_max);
  return signal_output;
}
int givePositionInPWM(int pos){
  pos = constrain(pos, -100, 100);
  int signal_min = 500;
  int signal_max = 2500;
  int signal_output = map(pos, -100, 100, signal_min, signal_max);
  return signal_output;
}
int givePowerInPWM(int power) {
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max);
  return signal_output;
}

void moveToPosition(Servo &servo, int targetPosition, int duration) {
  int currentPosition = servo.readMicroseconds();
  int stepSize = 1;  // Small fixed step size in microseconds
  int totalSteps = abs(targetPosition - currentPosition) / stepSize;
  int stepDelay = duration / totalSteps;  // Calculate delay based on the total number of steps

  if (currentPosition < targetPosition) {
    for (int i = 0; i < totalSteps; i++) {
      currentPosition += stepSize;
      servo.writeMicroseconds(currentPosition);
      delay(stepDelay);
    }
  } else {
    for (int i = 0; i < totalSteps; i++) {
      currentPosition -= stepSize;
      servo.writeMicroseconds(currentPosition);
      delay(stepDelay);
    }
  }

  // Ensure final position is set accurately
  servo.writeMicroseconds(targetPosition);
}

//Sensor Methods
//---------------------------------------------------------------------------------------

bool motionDetected (){
  pinStatePrevious = pinStateCurrent; // store old state
  pinStateCurrent = digitalRead(MOTION_SENSOR_PIN);   // read new state

  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {   // pin state change: LOW -> HIGH
    Serial.println("Motion detected!");
    return true;
  }
  else
  if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {   // pin state change: HIGH -> LOW
    Serial.println("Motion stopped!");
    return false;
  }
}

bool IRDetected (){
  IRcurrentState = digitalRead(IR_DETECTION_PIN);

  if (IRlastState == HIGH && IRcurrentState == LOW)
    Serial.println("Thing detected");
  else if (IRlastState == LOW && IRcurrentState == HIGH)
    Serial.println("Nothing detected");
  
  delay(50);

  IRlastState = IRcurrentState;
}

void ledColor(int R, int G, int B){
  analogWrite(R_PIN, R);
  analogWrite(G_PIN,  G);
  analogWrite(B_PIN, B);
}

void fireUltrasound() {
  // 1. TAKING MULTIPLE MEASUREMENTS AND STORE IN AN ARRAY
  for (int sample = 0; sample < 20; sample++) {
    filterArray[sample] = ultrasonicMeasure();
    delay(30); // to avoid untrasonic interfering
  }

  // 2. SORTING THE ARRAY IN ASCENDING ORDER
  for (int i = 0; i < 19; i++) {
    for (int j = i + 1; j < 20; j++) {
      if (filterArray[i] > filterArray[j]) {
        float swap = filterArray[i];
        filterArray[i] = filterArray[j];
        filterArray[j] = swap;
      }
    }
  }

  // 3. FILTERING NOISE
  // + the five smallest samples are considered as noise -> ignore it
  // + the five biggest  samples are considered as noise -> ignore it
  // ----------------------------------------------------------------
  // => get average of the 10 middle samples (from 5th to 14th)
  double sum = 0;
  for (int sample = 5; sample < 15; sample++) {
    sum += filterArray[sample];
  }

  distance = sum / 10;
}

float ultrasonicMeasure() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  float distance_cm = 0.017 * duration_us;

  return distance_cm;
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}