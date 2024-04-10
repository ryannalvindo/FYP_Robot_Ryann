// Start the simulation, click on the MPU6050 part, and
// change the x/y/z values to see changes in the plotter.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
double angle;
double start_time;
double oldGyro;

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
  angle = 0.0;
  start_time = 0.0;
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
//  Serial.print("Acceleration X: ");
//  Serial.print(a.acceleration.x);
//  Serial.print(", Y: ");
//  Serial.print(a.acceleration.y);
//  Serial.print(", Z: ");
//  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

//  Serial.print("Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.println(" degC");

  double gyroRate = g.gyro.z - oldGyro;
  Serial.println("oldGyro: " + String(oldGyro));
  
  angle = angle + (gyroRate * (double(millis()) - start_time)/1000 * 180 / PI);
  start_time = double(millis());
  oldGyro = g.gyro.z;

  Serial.println("start time: " + String(start_time));
  Serial.println("gyroRate: " + String(gyroRate));

  Serial.print("Angle is: ");
  Serial.print(angle);
  Serial.println(" deg");
  

//  delay(10);
}
