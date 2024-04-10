// Libraries
#include <Arduino.h>

// Define pin numbers
const int encoderPinA_1 = 2;    // Encoder pin A for motor 1
const int encoderPinB_1 = 3;    // Encoder pin B for motor 1
const int motorPin1A = 8;       // Motor 1 control pin A
const int motorPin1B = 9;       // Motor 1 control pin B
const int encoderPinA_2 = 6;    // Encoder pin A for motor 2
const int encoderPinB_2 = 7;    // Encoder pin B for motor 2
const int motorPin2A = 10;      // Motor 2 control pin A
const int motorPin2B = 11;      // Motor 2 control pin B

// Variables
volatile int encoderPos_1 = 0;      // Current encoder position for motor 1
volatile int encoderLastPos_1 = 0;  // Previous encoder position for motor 1
volatile int encoderPos_2 = 0;      // Current encoder position for motor 2
volatile int encoderLastPos_2 = 0;  // Previous encoder position for motor 2
boolean reversePlayback = false;    // Flag to indicate reverse playback

void setup() {
  // Set encoder pins as inputs with pullup resistors
  pinMode(encoderPinA_1, INPUT);
  pinMode(encoderPinB_1, INPUT);
  pinMode(encoderPinA_2, INPUT_PULLUP);
  pinMode(encoderPinB_2, INPUT_PULLUP);

  // Set motor control pins as outputs
  pinMode(motorPin1A, OUTPUT);
  pinMode(motorPin1B, OUTPUT);
  pinMode(motorPin2A, OUTPUT);
  pinMode(motorPin2B, OUTPUT);

  // Initialize motor pins to LOW
  digitalWrite(motorPin1A, LOW);
  digitalWrite(motorPin1B, LOW);
  digitalWrite(motorPin2A, LOW);
  digitalWrite(motorPin2B, LOW);
  
  // reverse
  Serial.begin(9600);
  pinMode(4,INPUT_PULLUP);
}

void loop() {
  // Record encoder values
  while (!reversePlayback) {
    updateEncoder_1();
    updateEncoder_2();
    // You can perform other tasks here if needed
    if(digitalRead(4) == 0){
      Serial.println("Start Reverse");
      startReversePlayback();
    }
  }

  // Perform reverse playback if flag is set
  if (reversePlayback) {
    // Move motor 1 in reverse direction based on encoder values
    for (int i = encoderPos_1; i >= 0; i--) {
      digitalWrite(motorPin1A, LOW);
      digitalWrite(motorPin1B, HIGH);
      delay(100); // Adjust delay as necessary
      digitalWrite(motorPin1A, LOW);
      digitalWrite(motorPin1B, LOW);
      delay(100); // Adjust delay as necessary
    }

    // Move motor 2 in reverse direction based on encoder values
    for (int i = encoderPos_2; i >= 0; i--) {
      digitalWrite(motorPin2A, LOW);
      digitalWrite(motorPin2B, HIGH);
      delay(10); // Adjust delay as necessary
      digitalWrite(motorPin2A, LOW);
      digitalWrite(motorPin2B, LOW);
      delay(10); // Adjust delay as necessary
    }

    // Reset flag after playback
    reversePlayback = false;
  }
}

// Function to update encoder position for motor 1
void updateEncoder_1() {
  int MSB = digitalRead(encoderPinA_1); // Most significant bit
  int LSB = digitalRead(encoderPinB_1); // Least significant bit
  int encoded = (MSB << 1) | LSB;       // Combine bits

  int sum = (encoderLastPos_1 << 2) | encoded; // Add new bits to the previous position

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011 ||
      sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000 ||
      sum == 0b1111 || sum == 0b1100 || sum == 0b1000 || sum == 0b0001 ||
      sum == 0b0000 || sum == 0b0101 || sum == 0b0111 || sum == 0b1010) {
    encoderPos_1--; // Decrement encoder position for clockwise rotation
  } else if (sum == 0b1111 || sum == 0b1100 || sum == 0b1000 || sum == 0b0001 ||
             sum == 0b0000 || sum == 0b0101 || sum == 0b0111 || sum == 0b1010 ||
             sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos_1++; // Increment encoder position for counterclockwise rotation
  }

  encoderLastPos_1 = encoded; // Update previous position
  Serial.println(encoded);
}

// Function to update encoder position for motor 2
void updateEncoder_2() {
  static int lastState_2 = 0;
  int state_2 = digitalRead(encoderPinA_2);
  if ((lastState_2 == LOW) && (state_2 == HIGH)) {
    int dir_2 = digitalRead(encoderPinB_2);
    encoderPos_2 += (dir_2 == LOW) ? -1 : 1;
  }
  lastState_2 = state_2;
}

// Function to initiate reverse playback
void startReversePlayback() {
  reversePlayback = true;
}
