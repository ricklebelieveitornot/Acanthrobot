/* Project: Arcanthrobot
 *  Author: Laneal Dickerson, Ricky Le, Michael Moloney
  - Partial Code Credit:  panerqiang@sunfounder.com
  - Date:  4/18/2019
  - Class: CSC 4110 Embedded Systems
  - Instructor: Dr. Michael Weeks
   -----------------------------------------------------------------------------
  - Overview: This project allows a robot using an Arduino Mega 2560 and a HC-SR04
    Sensor to "Autonomously" Move around and avoid obstacles. 
  - Usage: Run the Code and Upload to Arduino Board. The Robot will begin to move on
    its own. 
 */

#include <Servo.h>    //Need Servo Library to Initialize and Use Servos
#include <FlexiTimer2.h>//Need FlexiTimer Library to control speed and timing of Servos

/* Servo Initialization */
Servo servo[4][3];
const int servo_pin[4][3] = { {7, 6, 5}, {9, 10, 8}, {3, 2, 4}, {12, 13, 11} };

/* Defined Constants for Size of Robot */
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;

/* Defined Constants for Movement */
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;

/* Defined Variables for Movement */
volatile float site_now[4][3];    //"Real-Time" Coordinates of Legs
volatile float site_expect[4][3]; //Expected Coordinates of Legs
float temp_speed[4][3];   //Speed of Servo Motors (Turn Speed), Re-calculated before every movement
float move_speed;     //Movement Speed
float speed_multiple = 1; //Movement Speed Multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //Automatic Rest Counter
const float KEEP = 255;
const float pi = 3.1415926;
/* Defined Constants for Turning */
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

/* Initialize Sensor and Sensor Variables */
const int trigPin = 22;
const int echoPin = 23;
long duration;
int distance;

void setup(){
  pinMode(trigPin, OUTPUT); // Set Trig Pin on Sensor to Output
  pinMode(echoPin, INPUT); // Set Echo Pin on Sensor to read Input
  Serial.begin(9600); // Serial Communication Baud Rate
  Serial.println("Robot starts initialization");
  set_site(0, x_default - x_offset, y_start + y_step, z_boot); // Initialize Default/Starting Coordinates
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      site_now[i][j] = site_expect[i][j];
    }
  }
  FlexiTimer2::set(20, servo_service); //Start Servo Timers
  FlexiTimer2::start();
  Serial.println("Servo service started");
  servo_attach(); //Initialize Servos
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");
  stand();
  delay(3000);
}

/* Function to Initialize Servos / Set Pins */
void servo_attach(void){
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
}

/* Function to Remove Servo Communication from Pins (For Documentation) */
void servo_detach(void){
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      servo[i][j].detach();
      delay(100);
    }
  }
}

/* Loop Function; Senses Surrounding and Avoids if Necessary; Constantly Moves Forward */
void loop(){
  sense();
  avoid();
  step_forward(1);
}

/* Senses using HCSR04 Sensor; Pulses for 2 Microseconds then Pulses a HIGH Signal for Echo to read back information */
void sense(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2; //Calculate Distance from the Return Time of the Sound Wave
  Serial.print("Distance: ");
  Serial.println(distance);
}

/* If Distance Calculated is Less than 20, Step Back and Check Left, if Object is still in the way, turn right */
void avoid(){
    if(distance <= 20){
      Serial.println("Obstacle Detected.");
      step_back(3);
      turn_left(3);
      Serial.println("Updating Sensors.");
      sense();
      if(distance <= 20){
        turn_right(6);
      }
        else{
          turn_left(3);
        }
      }
}

/* Standing Function, Used to have a default starting stance */
void stand(void){
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++){
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
  Serial.println("Standing.");
}

/* Turn Left Function */
/* Function moves Legs 1 and 3, and then waits for Servos to finish before moving 2 and 4, Simulating Turning Movement */
void turn_left(unsigned int step){
  move_speed = spot_turn_speed;
  while (step-- > 0){
    if (site_now[3][1] == y_start){
      set_site(3, x_default + x_offset, y_start, z_up); //Move Leg 0 and 2
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default); 
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      set_site(0, x_default + x_offset, y_start, z_up); //Move Leg 1 and 3
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
  Serial.println("Turning Left.");
}

/* Turning Right Function */
/* Does the Reverse order of turn_left() */
void turn_right(unsigned int step){
  move_speed = spot_turn_speed;
  while (step-- > 0){
    if (site_now[2][1] == y_start){
      set_site(2, x_default + x_offset, y_start, z_up); //Move Legs 1 and 3
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      set_site(1, x_default + x_offset, y_start, z_up); //Move Legs 0 and 2
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
  Serial.println("Turning Right.");
}

/* Move Forward */
/* Imitates Spider Movement going Forward */
void step_forward(unsigned int step){
  move_speed = leg_move_speed;
  while (step-- > 0){
    if (site_now[2][1] == y_start){
      set_site(2, x_default + x_offset, y_start, z_up); //Move Legs 0 and 1
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      set_site(0, x_default + x_offset, y_start, z_up); //Move Legs 2 and 3
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
  Serial.println("Stepping Forward.");
}

/*Move Backwards Function */
/* Does the Reverse of step_forward() */
void step_back(unsigned int step){
  move_speed = leg_move_speed;
  while (step-- > 0){
    if (site_now[3][1] == y_start){
      set_site(3, x_default + x_offset, y_start, z_up); //Move Legs 0 and 3
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      set_site(1, x_default + x_offset, y_start, z_up); //Moves Legs 1 and 2
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/* Timer for Servos */
/* when set_site() is called, this function moves the servo to end point in a line */
void servo_service(void){
  sei();
  static float alpha, beta, gamma;
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }
    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }
  rest_counter++;
}

/*Function to Move Servo in a Synchronous Motion*/
void set_site(int leg, float x, float y, float z){
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/* Function to block other movement functions until the previous one is finished */
void wait_reach(int leg){
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*Function to block other movement functions until All other functions are finished */
void wait_all_reach(void){
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/* Converts Cartesian Coordinates to Polar Coordinates (since Servos are in Cartesian 3D Space but turn in Polar Space */
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z){
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/* Converts Polar Coordinates to Servo Turn Distance */
void polar_to_servo(int leg, float alpha, float beta, float gamma){
  if (leg == 0){
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1){
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2){
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3){
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}
