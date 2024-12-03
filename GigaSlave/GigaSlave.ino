#include <Wire.h>
#include <Servo.h>

Servo rotor_ESC;
Servo pan;
Servo tilt;
Servo feed;
Servo LDrive;
Servo RDrive;

#define ROTOR_ESC_PIN 2
#define FEED_PIN 3
#define PAN_PIN 4
#define TILT_PIN 5
#define LDRIVE_PIN 6
#define RDRIVE_PIN 7

bool stopManual = true;
int currentLDrivePower = 0;  // Store the current power levels of the motors
int currentRDrivePower = 0;

// Struct to hold 10 channels (6 int channels and 4 boolean channels)
struct ChannelData {
  int16_t channelValues[6];  // Channels 0-5
  uint8_t switches[4];       // Channels 6-9 (boolean switches)
};

ChannelData receivedData;    // Struct to store the received data

// Global variables to store the current channel and switch states
int16_t channels[6];  // Channels 0-5
bool switches[4];     // Switches 6-9

void receiveEvent(int howMany) {
  if (howMany == sizeof(receivedData)) {
    // Read the received bytes into the struct
    Wire.readBytes((char*)&receivedData, sizeof(receivedData));

    // Update the global channel and switch variables
    for (int i = 0; i < 6; i++) {
      channels[i] = receivedData.channelValues[i];  // Store channel data in global array
      Serial.print("Ch: ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(channels[i]);
      Serial.print(" ");
    }

    for (int i = 0; i < 4; i++) {
      switches[i] = receivedData.switches[i];  // Store switch data in global array
      Serial.print("Sw: ");
      Serial.print(i + 6);
      Serial.print(": ");
      Serial.print(switches[i]);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Error: Incorrect data length received");
  }
}

void setup() {
  delay(1000);
  Serial.begin(115200);         // USB serial for debugging
  Wire.begin(8);                // Join I2C bus with address 8 as slave
  Wire.onReceive(receiveEvent); // Register receive event handler
  Serial.println("Giga R1 ready to receive data");

  rotor_ESC.attach(ROTOR_ESC_PIN);
  pan.attach(PAN_PIN);
  tilt.attach(TILT_PIN);
  LDrive.attach(LDRIVE_PIN);
  RDrive.attach(RDRIVE_PIN);
  feed.attach(FEED_PIN);
}

void loop() {
  // Stop all servos if Switch 3 or 0 are not in the off/on state (Switch 3 should be true and Switch 0 false)
  if (!(switches[3] && !switches[0]) && stopManual) { 
    stopAllServos();
    stopManual = false;  // Ensures the stop is only triggered once
    Serial.println("Servos stopped");
  }

  // Manual control when Switch 3 is on and Switch 0 is off
  if (switches[3] && !switches[0]) {
    stopManual = false;  // Reset stop flag, allowing other systems to take over
    Serial.println("Manual control enabled");

    tilt.writeMicroseconds(givePowerInPWM(channels[2]));  // Channel 2 for tilt
    pan.writeMicroseconds(givePositionInPWM(channels[3])); // Channel 3 for pan
    rotor_ESC.writeMicroseconds(rotorPWM(channels[4]));   // Channel 4 for rotor
    
    // Apply feed deadzone (ignore small values)
    if (abs(channels[5]) > 15) {
      feed.write(feedPWM(channels[5]));
    } else {
      feed.write(feedPWM(0));  // Stop feed motor if within deadzone
    }

    int targetLDrivePower = constrain(map(channels[1] + channels[0], -100, 100, -40, 40), -40, 40);  // Y + X
    int targetRDrivePower = constrain(map(channels[1] - channels[0], -100, 100, -40, 40), -40, 40);  // Y - X
    
    // Ramp the motor speeds smoothly
    currentLDrivePower = rampSpeed(currentLDrivePower, targetLDrivePower, 2000);  // Adjust the 5 for slower/faster ramping
    currentRDrivePower = rampSpeed(currentRDrivePower, targetRDrivePower, 2000);

    LDrive.writeMicroseconds(givePowerInPWM(currentLDrivePower));
    RDrive.writeMicroseconds(givePowerInPWM(currentRDrivePower));
  }
}

void stopAllServos() {
  rotor_ESC.writeMicroseconds(1500);  // Stop rotor (neutral PWM)
  pan.writeMicroseconds(1500);        // Center pan
  tilt.writeMicroseconds(1500);       // Center tilt
  feed.writeMicroseconds(1500);       // Stop feed
  LDrive.writeMicroseconds(1500);     // Stop LDrive
  RDrive.writeMicroseconds(1500);     // Stop RDrive
}

int rotorPWM(int speed) {
  speed = constrain(speed, -10, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(speed, -10, 100, signal_min, signal_max);
  return signal_output;
}

int givePositionInPWM(int pos) {
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

int feedPWM(int power) {
  power = constrain(power, -100, 100);
  int signal_min = 1000;
  int signal_max = 2000;
  int signal_output = map(power, -100, 100, signal_min, signal_max);
  return signal_output;
}

// Function to gradually adjust speed
int rampSpeed(int currentSpeed, int targetSpeed, int increment) {
    if (currentSpeed < targetSpeed) {
        currentSpeed += increment;
        if (currentSpeed > targetSpeed) {
            currentSpeed = targetSpeed;
        }
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= increment;
        if (currentSpeed < targetSpeed) {
            currentSpeed = targetSpeed;
        }
    }
    return currentSpeed;
}