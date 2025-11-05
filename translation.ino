#include<stdio.h>
#include <wifimodule.ino>
#include <enes100.ino>


void setMotor(int IN1, int IN2, int pwmPIN, bool positive);
void relmotion(extern float heading, char axis, float d)
extern float idealx;
extern float idealy;
// at some point we need to move this to main 
void loop(){
    if (!Enes100.isVisible()) {
        stop();
// send message that the vision system is not working 
    }
    float x = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
    float y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
    float heading = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
    float goToX = idealx;//put desired x coordinate here
    float goToY = idealy;//put desired x coordinate here
    float deltaX = goToX - x;
    float deltaY = goToY - y;


    if (abs(deltaX) > 0.1) relmotion(heading, 'x', deltaX);
    else if (abs(deltaY) > 0.1) relmotion(heading, 'y', deltaY);
  // Add delay as needed to prevent command spam
  delay(100);
}


// Motor A pins
int A_IN1 = //pin number to positive H-bridge;
int A_IN2 = //pin number to negative H-bridge;
int A_PWM = //pin number for the power input(controlls speed);  // must be a PWM-capable pin

// Motor B pins
int B_IN1 = //pin number to positive H-bridge;
int B_IN2 = //pin number to negative H-bridge;;
int B_PWM = //pin number for the power input(controlls speed);  // must be a PWM-capable pin

// Motor C pins
int C_IN1 = //pin number to positive H-bridge;
int C_IN2 = //pin number to negative H-bridge;;
int C_PWM = //pin number for the power input(controlls speed);  // must be a PWM-capable pin

// Motor D pins
int D_IN1 = //pin number to positive H-bridge;
int D_IN2 = //pin number to negative H-bridge;;
int D_PWM = //pin number for the power input(controlls speed);  // must be a PWM-capable pin

const int SPEED = 150; // Change this value as needed(changes the pwm)

void setup() {
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(A_PWM, OUTPUT);

  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  pinMode(C_IN1, OUTPUT);
  pinMode(C_IN2, OUTPUT);
  pinMode(C_PWM, OUTPUT);

  pinMode(D_IN1, OUTPUT);
  pinMode(D_IN2, OUTPUT);
  pinMode(D_PWM, OUTPUT);
}



void setMotor(int IN1, int IN2, int pwmPIN, bool positive) {
  digitalWrite(IN1, positive ? HIGH : LOW);
  digitalWrite(IN2, positive ? LOW : HIGH);
  analogWrite(pwmPIN, SPEED);
}

void go(){
    setMotor(A_IN1, A_IN2, A_PWM, true);
    setMotor(B_IN1, B_IN2, B_PWM, true);
    setMotor(C_IN1, C_IN2, C_PWM, true);
    setMotor(D_IN1, D_IN2, D_PWM, true);
}

void back(){
    setMotor(A_IN1, A_IN2, A_PWM, false);
    setMotor(B_IN1, B_IN2, B_PWM, false);
    setMotor(C_IN1, C_IN2, C_PWM, false);
    setMotor(D_IN1, D_IN2, D_PWM, false);
}

void left(){
    setMotor(A_IN1, A_IN2, A_PWM, false);
    setMotor(B_IN1, B_IN2, B_PWM, true);
    setMotor(C_IN1, C_IN2, C_PWM, true);
    setMotor(D_IN1, D_IN2, D_PWM, false);
}

void right(){
    setMotor(A_IN1, A_IN2, A_PWM, true);
    setMotor(B_IN1, B_IN2, B_PWM, false);
    setMotor(C_IN1, C_IN2, C_PWM, false);
    setMotor(D_IN1, D_IN2, D_PWM, true);
}

void cc(){
    setMotor(A_IN1, A_IN2, A_PWM, true);
    setMotor(B_IN1, B_IN2, B_PWM, false);
    setMotor(C_IN1, C_IN2, C_PWM, true);
    setMotor(D_IN1, D_IN2, D_PWM, false);
}

void ccw(){
    setMotor(A_IN1, A_IN2, A_PWM, false);
    setMotor(B_IN1, B_IN2, B_PWM, true);
    setMotor(C_IN1, C_IN2, C_PWM, false);
    setMotor(D_IN1, D_IN2, D_PWM, true);
}


void stop() {
    // Set all PWM values to 0 to stop motors
    analogWrite(A_PWM, 0);
    analogWrite(B_PWM, 0);
    analogWrite(C_PWM, 0);
    analogWrite(D_PWM, 0);
}

// Relational movement based on heading and coordinate axis
void relmotion(float heading, char axis, float delta) {
  int headingIdx = -1, moveIdx = -1;

  // Map heading to index (allowing some tolerance)
  // Heading: N = 0, E = 1, S = 2, W = 3
  if (abs(heading) < 0.2) headingIdx = 0;            // North
  else if (abs(heading - 1.57) < 0.2) headingIdx = 1; // East
  else if (abs(abs(heading) - 3.14) < 0.2) headingIdx = 2; // South
  else if (abs(heading + 1.57) < 0.2) headingIdx = 3; // West
  
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

  // If both indices are valid, perform movement
  if (headingIdx != -1 && moveIdx != -1) {
    moveRelative[headingIdx][moveIdx]();
  }
}

bool isVisible() {
    float x = Enes100.getX();
    float y = Enes100.getY();
    // If either x or y is -1, marker is not visible
    if (x == -1 || y == -1) {
        return false;
    }
    return true;
}