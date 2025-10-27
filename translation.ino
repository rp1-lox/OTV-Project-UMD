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


void go();
void back();
void left();
void right();

void go(){
    /*
    +i +j
    
    +j +i
    */
}

void back(){
    /*
    -i -j
    -j -i
    */
}

//etc 