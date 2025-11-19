#include <Enes100.h>
#include <math.h>
  





 /* X = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
  Y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
  H = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
  V = Enes100.isVisible(); // Is your aruco visible? True or False.
  
  translate(curX, curY, idealX, idealY);*/


void translate(float curX, float curY, float idealX, float idealY){
    if (abs(idealX-curX) > 0.1){
      relmotion(H, 'x', (idealX-curX));
      Enes100.println("y");
    } 
    else if (abs(idealY-curY) > 0.1){
      relmotion(H, 'y', idealY-curY);
      Enes100.println("x");
    } 
    delay(200);
}

void setMotor(int IN1, int IN2, int pwmPIN, int speed, bool positive) {
  digitalWrite(IN1, positive ? HIGH : LOW);
  digitalWrite(IN2, positive ? LOW : HIGH);
  analogWrite(pwmPIN, speed);
}
void back(){
    setMotor(A_IN1, A_IN2, A_ENA, SA, true);
    setMotor(B_IN3, B_IN4, B_ENB, SB, false);
    setMotor(C_IN5, C_IN6, C_ENC, SC, false);
    setMotor(D_IN7, D_IN8, D_END, SD, true);
}

void go(){
    setMotor(A_IN1, A_IN2, A_ENA, SA, false);
    setMotor(B_IN3, B_IN4, B_ENB, SB, true);
    setMotor(C_IN5, C_IN6, C_ENC, SC, true);
    setMotor(D_IN7, D_IN8, D_END, SD, false);
}

void left(){
    setMotor(A_IN1, A_IN2, A_ENA, SA, true);
    setMotor(B_IN3, B_IN4, B_ENB, SB, true);
    setMotor(C_IN5, C_IN6, C_ENC, SC, false);
    setMotor(D_IN7, D_IN8, D_END, SD, false);
}

void right(){
    setMotor(A_IN1, A_IN2, A_ENA, SA, false);
    setMotor(B_IN3, B_IN4, B_ENB, SB, false);
    setMotor(C_IN5, C_IN6, C_ENC, SC, true);
    setMotor(D_IN7, D_IN8, D_END, SD, true);
}

void cw(){
    setMotor(A_IN1, A_IN2, A_ENA, SA, false);
    setMotor(B_IN3, B_IN4, B_ENB, SB, true);
    setMotor(C_IN5, C_IN6, C_ENC, SC, false);
    setMotor(D_IN7, D_IN8, D_END, SD, true);
}

void ccw(){
    setMotor(A_IN1, A_IN2, A_ENA, SA, true);
    setMotor(B_IN3, B_IN4, B_ENB, SB, false);
    setMotor(C_IN5, C_IN6, C_ENC, SC, true);
    setMotor(D_IN7, D_IN8, D_END, SD, false);
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
    delay(300);
  }
}
