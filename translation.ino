#include <stdio.h>
#include "waterboys.h"
#include "Enes100.h"




// at some point we need to move this to main 
void setMotor(int IN1, int IN2, int pwmPIN, bool positive);
void relmotion(extern float heading, char axis, float d)

//float targettolerance = 0.05

void translate(float curX, float curY, float idealX, float idealY){
    if (abs(idealX-curX) > 0.1/*tolerance*/) relmotion(H, 'x', (idealX-curX));
    else if (abs(idealY-curY) > 0.1/*tolerance*/) relmotion(H, 'y', idealY-curY);
}

  
// Motor A (HB1)(OUT1/OUT2)(Front Right)
const int A_IN1 = 22;
const int A_IN2 = 23;
const int A_ENA = 4;   // PWM

// Motor B (HB1)(OUT3/OUT4)(Back Right)
const int B_IN3 = 24;
const int B_IN4 = 25;
const int B_ENB = 5;   // PWM

// Motor C (HB2)(OUT1/OUT2)(Back Left)
const int C_IN5 = 26;
const int C_IN6 = 27;
const int C_ENC = 6;   // PWM

// Motor D (HB2)(OUT3/OUT4) (Front Left)
const int D_IN7 = 28;
const int D_IN8 = 29;
const int D_END = 7;   // PWM

const int SPEED = 150; // Change this value as needed(changes the pwm)

void setup() {

  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(A_ENA, OUTPUT);

  pinMode(B_IN3, OUTPUT);
  pinMode(B_IN4, OUTPUT);
  pinMode(B_ENB, OUTPUT);

  pinMode(C_IN5, OUTPUT);
  pinMode(C_IN6, OUTPUT);
  pinMode(C_ENC, OUTPUT);

  pinMode(D_IN7, OUTPUT);
  pinMode(D_IN8, OUTPUT);
  pinMode(D_END, OUTPUT);

  motorsOff();

}



void setMotor(int IN1, int IN2, int pwmPIN, bool positive) {
  digitalWrite(IN1, positive ? HIGH : LOW);
  digitalWrite(IN2, positive ? LOW : HIGH);
  analogWrite(pwmPIN, SPEED);
}

void go(){
    setMotor(A_IN1, A_IN2, A_ENA, true);
    setMotor(B_IN3, B_IN4, B_ENB, false);
    setMotor(C_IN5, C_IN6, C_ENC, false);
    setMotor(D_IN7, D_IN8, D_END, true);
}

void back(){
    setMotor(A_IN1, A_IN2, A_ENA, false);
    setMotor(B_IN3, B_IN4, B_ENB, true);
    setMotor(C_IN5, C_IN6, C_ENC, true);
    setMotor(D_IN7, D_IN8, D_END, false);
}

void left(){
    setMotor(A_IN1, A_IN2, A_ENA, true);
    setMotor(B_IN3, B_IN4, B_ENB, true);
    setMotor(C_IN5, C_IN6, C_ENC, false);
    setMotor(D_IN7, D_IN8, D_END, false);
}

void right(){
    setMotor(A_IN1, A_IN2, A_ENA, false);
    setMotor(B_IN3, B_IN4, B_ENB, false);
    setMotor(C_IN5, C_IN6, C_ENC, true);
    setMotor(D_IN7, D_IN8, D_END, true);
}

void cw(){
    setMotor(A_IN1, A_IN2, A_ENA, false);
    setMotor(B_IN3, B_IN4, B_ENB, false);
    setMotor(C_IN5, C_IN6, C_ENC, false);
    setMotor(D_IN7, D_IN8, D_END, false);
}

void ccw(){
    setMotor(A_IN1, A_IN2, A_ENA, true);
    setMotor(B_IN3, B_IN4, B_ENB, true);
    setMotor(C_IN5, C_IN6, C_ENC, true);
    setMotor(D_IN7, D_IN8, D_END, true);
}


void stop() {
    // Set all PWM values to 0 to stop motors
    analogWrite(A_PWA, 0);
    analogWrite(B_PWB, 0);
    analogWrite(C_PWC, 0);
    analogWrite(D_PWD, 0);
}

// Relational movement based on heading and coordinate axis
void relmotion(float heading, char axis, float delta){
  int headingIdx = -1, moveIdx = -1;

  // Map heading to index (allowing some tolerance)
  // Heading: N = 0, E = 1, S = 2, W = 3
  if (abs(heading) < 0.2) headingIdx = 0;            // North
  else if (abs(heading - 1.57) < 0.2) headingIdx = 1; // East
  else if (abs(abs(heading) - 3.14) < 0.2) headingIdx = 2; // South
  else if (abs(heading + 1.57) < 0.2) headingIdx = 3; // West
  
//else if orient to heading function

  // Map axis and movement to index
  if (axis == 'x') {
    if (delta > 0.1) moveIdx = 3;    // Move right
    else if (delta < -0.1) moveIdx = 2; // Move left
  } else if (axis == 'y') {
    if (delta > 0.1) moveIdx = 0;    // Move forward
    else if (delta < -0.1) moveIdx = 1; // Move backward
  }

  // Movement matrix, indexed [heading][move]
  void (*moveRelative[4][4])() = {
    { go, back, left, right }, // Heading N
    { right, left, go, back }, // Heading E
    { back, go, right, left }, // Heading S
    { left, right, back, go }  // Heading W
  };
  
  //int movetime = ;  //Defines the movetime for a movement moving toward idealx idealy
  // If both indices are valid, perform movement
  if (headingIdx != -1 && moveIdx != -1) {
    moveRelative[headingIdx][moveIdx]();
    delay(300);
  }
}
