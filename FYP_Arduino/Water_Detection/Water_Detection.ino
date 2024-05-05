#include "water_detection.h"

void setup()
{
    // Initialize Serial communication
    Serial.begin(115200);
    setupWaterDetect(15);

}

void loop()
{

    waterDetection();
   
    // if (analogRead(waterDetectorPin) > 3){
    //     Serial.println("Water\n\n\n\n\n");
    // }
    // Serial.println(analogRead(waterDetectorPin));
    delay(1000);


}
