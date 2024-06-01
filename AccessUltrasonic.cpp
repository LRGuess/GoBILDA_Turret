#include <Arduino.h>
#include "AccessUltrasonic.h"

float filterArray[20]; // array to store data samples from sensor
float distance; // store the distance from sensor

int trigPin;
int echoPin;

static float fireUltrasound() {
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
  return distance;
}

void configurePins(int TriggerPin, int EchoPin){
  // configure the trigger and echo pins to output mode
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  trigPin = TriggerPin;
  echoPin = echoPin;
}

private:
  float ultrasonicMeasure() {
    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // measure duration of pulse from ECHO pin
    float duration_us = pulseIn(echoPin, HIGH);

    // calculate the distance
    float distance_cm = 0.017 * duration_us;

    return distance_cm;
  }

