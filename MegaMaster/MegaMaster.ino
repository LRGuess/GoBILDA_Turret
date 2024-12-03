#include <Wire.h>
#include <IBusBM.h>


#define LASER_PIN 22

IBusBM FSReceiver;

// Struct to hold 10 channels (6 int channels and 4 boolean channels)
struct ChannelData {
  int16_t channelValues[6];  // Channels 0-5
  uint8_t switches[4];       // Channels 6-9 (boolean switches)
};

ChannelData dataToSend;

void setup() {
  Wire.begin();                // Initialize I2C as master
  Serial.begin(115200);        // For debugging
  Serial.println("Mega ready to send data");

  FSReceiver.begin(Serial1);   // Initialize iBus on Serial1

  pinMode(LASER_PIN, OUTPUT);
}

void loop() {

  digitalWrite(LASER_PIN, HIGH);
  // Fill the struct with channel data
  for (int i = 0; i < 6; i++) {
    dataToSend.channelValues[i] = readChannel(i, -100, 100, 0); // Channels 0-5
  }
  
  for (int i = 0; i < 4; i++) {
    dataToSend.switches[i] = readSwitch(i + 6, false);  // Channels 6-9 (switches)
  }

  // Begin transmission to slave device with address 8
  Wire.beginTransmission(8);
  Wire.write((uint8_t*)&dataToSend, sizeof(dataToSend));  // Send the struct as bytes
  Wire.endTransmission();      // End transmission

  // Debugging output
  Serial.print("Message sent: ");
  for (int i = 0; i < 6; i++) {
    Serial.print("Ch");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(dataToSend.channelValues[i]);
    Serial.print(" ");
  }
  for (int i = 0; i < 4; i++) {
    Serial.print("Sw");
    Serial.print(i + 6);
    Serial.print(": ");
    Serial.print(dataToSend.switches[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  delay(125);                 // Send every eigth of a second
}

// Read the channel value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = FSReceiver.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);  // Map the channel value to desired range
}

// Read the switch value (boolean) and invert it
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  bool switchValue = (ch > 50);
  return !switchValue;  // Invert the switch value
}

