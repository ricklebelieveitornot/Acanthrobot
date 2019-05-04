//Initial Setup of Robot (setting servos default PHYSICAL locations at 90 degrees)

#include <Servo.h>   

Servo servo[4][3];

//define servos' ports
const int servo_pin[4][3] = { {7, 6, 5}, {9, 10, 8}, {3, 2, 4}, {12, 13, 11} };

void setup(){
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      servo[i][j].attach(servo_pin[i][j]);
      delay(20);
    }
  }
}

void loop(void){
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      servo[i][j].write(90);
      delay(20);
    }
  }
}
