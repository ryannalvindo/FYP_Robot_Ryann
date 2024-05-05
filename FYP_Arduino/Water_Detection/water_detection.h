#ifndef WATER_DETECTION_H
#define WATER_DETECTION_H

#define waterDetectorPin 14
#define waterThreshold 3

void setupWaterDetect(int detectorPin = waterDetectorPin);
bool waterDetection(int threshold = waterThreshold);

#endif