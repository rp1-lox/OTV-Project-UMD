#include <Adafruit-Motor-Shield-library-master\Adafruit-Motor-Shield-library-master\AFMotor.h>
#include <ENES100ArduinoLibrary-master\ENES100ArduinoLibrary-master\src\Enes100.h>

void setMotor(int IN1, int IN2, int pwmPIN, bool positive);
void relmotion(float heading, char axis, float d)

void loop(){
    float x = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
    float y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
    float heading = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
    float goToX = 4;//put desired x coordinate here
    float goToY = 4;//put desired x coordinate here
    float deltaX = goToX - x;
    float deltaY = goToY - y;
    if (-.1< deltaX <.1) relmotion(heading,'x', goToX);
    if (-.1< deltaY <.1) relmotion(heading,'y', goToY);



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



/*
if deltaX is positive --> left
if deltaX is negative --> right
if deltaY is positive --> go
if deltaY is negative --> back


if heading == 0
    north == go(), south == back(), west == left(), east == right()
if heading ~~ 1.5
    north == right(), south == left(), west == go(), east == back()
if heading ~~ -1.5
    north == left(), south == right(), west == back(), east == go()
if heading ~~ abv 3.1
    north == back(), south == go(), west == right(), east == go()



input            output
delta X and Y    2.as long as the x position is not within a margin of error, continue 
heading           
*/



void relmotion(float heading, char axis, float d){
    int headingIdx, moveIdx;
    void (*moveRelative[4][4])() = {
        //go,     back,       left,       right
        { go,     back,       left,       right }, // Heading N
        { right,  left,       go,         back  }, // Heading E
        { back,   go,         right,      left  }, // Heading S
        { left,   right,      back,       go    }  // Heading W
    };

    
// Translate heading to index
  if      (-0.1 < heading < 0.1) headingIdx = 0;
  else if (-1.4 < heading <-1.6) headingIdx = 1;
  else if ( 3.0 < heading < 3.2) headingIdx = 2;
  else if ( 1.4 < heading < 1.6) headingIdx = 3;
  else return;

// Translate desired direction to index
if (axis == 'x'){
  if        (x > 0) moveIdx = 0;
  else if   (x < 0) moveIdx = 1;
  else return
}
else if (axis == 'y'){
  else if   (y < 0) moveIdx = 2;
  else if   (y > 0) moveIdx = 3;
  else return;
}

  moveRelative[headingIdx][moveIdx]();

}









