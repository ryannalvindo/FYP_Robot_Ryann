#include <Arduino.h>

#define waterDetectorPin 14
#define waterThreshold 3


int detectorPinout;



/* 
The setup of the water detection
Place this in the void setup
Requirement: detectorPin must be Analogue pin
If the detectorPin is not declared, it will be using pin number 14
*/
void setupWaterDetect(int detectorPin = waterDetectorPin ){
    
    pinMode(detectorPin, OUTPUT);
    // save the detector pin number
    detectorPinout = detectorPin;
}


/*
The main process of the water detection
Place this in the void loop
Requirement: threshold must be greater than 1, 
however this value is evaluate based on trail and error
If the threshold is not declared, the threshold will be 3

Function will return true of false based on water availability
*/
bool waterDetection(int threshold = waterThreshold){
   
    if(analogRead(detectorPinout) > threshold){
        return true;
    }
    return false;
}