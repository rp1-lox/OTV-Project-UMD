/*
There are a couple of things that need to be input into each function first:
    1. power
        - use the equations that we got in class today but we might need to do some
          more math by finding the resistance?? not so sure this might be done better with testing 
        - lets use the variable pwm as the power.
            - this should be an int between 0-255 but chat gpt said something about scaling it down
              between -1, 0
    2. pin #
    3. to get negative spin use the other pin in the h-bridge


*/
#include <Adafruit-Motor-Shield-library-master\Adafruit-Motor-Shield-library-master\AFMotor.h>
#include <ENES100ArduinoLibrary-master\ENES100ArduinoLibrary-master\src\Enes100.h>

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

void setMotor();
void go();
void back();
void left();
void right();
void cw():
void ccw():

void setMotor(int IN1, int IN2, int pwmPIN, bool positive){
    if(positive){
        IN1 = HIGH;
        IN2 = LOW;
    }
    else{
        IN1 = LOW;
        IN2 = HIGH;
    }
    pwnPIN = SPEED; //speed will be a constant we define outside CONST SPEED = x
}

void go(){
    /*
    +i +j
    
    +j +i
    */
    setMotor(A_IN1, A_IN2, A_PWM, true);
    setMotor(B_IN1, B_IN2, B_PWM, true);
    setMotor(C_IN1, C_IN2, C_PWM, true);
    setMotor(D_IN1, D_IN2, D_PWM, true);
}

void back(){
    /*
    -i -j
    -j -i
    */
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


// This function picks the correct maneuver based on heading and movement command
void moveRelative(char heading, String movement) {
  // Table:           movement      0 = go/forward, 1 = back/backward, 2 = left, 3 = right
  // Heading Index:   0 = N, 1 = E, 2 = S, 3 = W
  void (*relativeMotions[4][4])() = {
    // go         back         left         right
    {   go,       back,        left,        right  }, // Heading N
    {  right,     left,        go,          back   }, // Heading E
    {  back,       go,         right,       left   }, // Heading S
    {  left,      right,       back,         go    }  // Heading W
  };

  int headingIdx, moveIdx;
  // Translate heading to index
  if      (heading == 'N') headingIdx = 0;
  else if (heading == 'E') headingIdx = 1;
  else if (heading == 'S') headingIdx = 2;
  else if (heading == 'W') headingIdx = 3;
  else return;

  // Translate movement to index
  if      (movement == "go")     moveIdx = 0;
  else if ( movement == "back")  moveIdx = 1;
  else if (movement == "left")   moveIdx = 2;
  else if (movement == "right")  moveIdx = 3;
  else return;

  // Execute the mapped movement
  (relativeMotions[headingIdx][moveIdx])();
}

void loop() {
  // EXAMPLES:
  moveRelative('N', "go");   // moves forward with heading N
  delay(1000);
  moveRelative('N', "left");      // moves left with heading N (i.e., world-west)
  delay(1000);
  moveRelative('E', "forward");   // moves right with heading E (i.e., world-east)
  delay(1000);
  moveRelative('E', "left");      // moves forward with heading E (i.e., world-north)
  delay(1000);
}
