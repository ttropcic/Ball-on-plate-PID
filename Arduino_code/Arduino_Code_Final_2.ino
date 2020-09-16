
#include <Servo.h>

Servo servoX;
Servo servoY;
float angleX = 34.0;
float angleY = 34.0;


void setup() {
  Serial.begin(19200);
  
  servoX.attach(9, 550, 2350);
  servoY.attach(11, 550, 2350);
 
  servoX.write(angleX);
  servoY.write(angleY);
}



void loop() {

  if(Serial.available() > 0) {
    angleX = Serial.readStringUntil(',').toFloat();  
    angleY = Serial.readStringUntil('\n').toFloat();      
    servoX.write(angleX);
    servoY.write(angleY);
   
    
  }

}
