#include <Enes100.h>

#include <Enes100.h>
#include <math.h>
const int pumpPin = 10;

#include <Wire.h>
#include "Adafruit_AS726x.h"

//create the object
Adafruit_AS726x ams;

//buffer to hold raw values
uint16_t sensorValues[AS726x_NUM_CHANNELS];

int trig=15;
int echo=14;

//updated by gavin 12.5.24

void translate(float curX, float curY, float idealX, float idealY);
void relmotion(float heading, char axis, float delta);
void orient(float idealH);
void backStraight(float targetHeading, int baseSpeed);
void goStraight(float targetHeading, int baseSpeed);
void leftStraight(float targetHeading, int baseSpeed);
void rightStraight(float targetHeading, int baseSpeed);
void stop();


// Motor A (HB1)(OUT1/OUT2)(Front Right)
int A_IN1 = 30;
int A_IN2 = 31;
int A_ENA = 4;   // PWM


// Motor B (HB1)(OUT3/OUT4)(Back Right)
int B_IN3 = 32;
int B_IN4 = 33;
int B_ENB = 3;   // PWM


// Motor C (HB2)(OUT1/OUT2)(Back Left)
int C_IN5 = 36;
int C_IN6 = 37;
int C_ENC = 12;   // PWM


// Motor D (HB2)(OUT3/OUT4) (Front Left)
int D_IN7 = 38;
int D_IN8 = 39;
int D_END = 13;   // PWM


const int SPEED = 150; // Change this value as needed(changes the pwm)


int SA = (1*SPEED);
int SB = (1*SPEED);
int SC = (0.81*SPEED);
int SD = (0.81*SPEED);


//define global variables here
 
float idealX = 0.35;
float idealY = 1.50;
float idealH = (M_PI / 2) + 0.2;
int step = 0;

// Add these global variables at the top with your other globals
float straightKp = 75.0;  // Heading correction gain for straight motion
float straightKd = 15.0;  // Derivative gain to dampen oscillation
float prevHeadingError = 0;
unsigned long lastStraightTime = 0;

char zone = 'a'; 
void setup(){
    //    initialize all pin numbers here
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
  Serial.begin(9600);
  analogWrite(pumpPin, 0);      //uncomment this and set pump anywhere from 0-255
  pinMode (trig,OUTPUT);
  pinMode (echo, INPUT);
  while(!Serial);
  if(!ams.begin()){
    Enes100.println("could not connect to color sensor! Please check your wiring.");
    while(1);
  }
//for wifi module
//test
    // Initialize Enes100 Library
    // Team Name, Mission Type, Marker ID, Room Number, Wifi Module TX Pin, Wifi Module RX Pin
    int txpin = 50;
    int rxpin = 51;
    Enes100.begin("Waterboys", WATER, 284, 1215, txpin, rxpin);
    // At this point we know we are connected.
    Enes100.println("Connected...");
}


/*

//not needed for testinf file!
//define all extern global variables here
float X;//x pos
float Y;//y pos
float H;//heading
bool V;//visibility

*/


/*
switch(step){
        case 1://determine the starting point
            starting_point = a_or_b(curY);
            step += 1;
            Enes100.print("Starting point: ");
            Enes100.println(starting_point);
            break; // ADDED BREAK
/*            
        case 2://rotate to proper orientation
            if(starting_point == 'a'){
                idealH = -1 * (M_PI / 2.0);
                Enes100.print("IdealH: ");
                Enes100.println(idealH);
            }
            else if(starting_point == 'b'){
                idealH = M_PI / 2.0;
                Enes100.print("IdealH: ");
                Enes100.println(idealH);
            }

            if(abs(idealH - H) >= 0.1){
                 orient(idealH);
                 Enes100.println("Orienting to idealH... ");
                 // No need to step+=1 and return here if orient contains a while(true) loop
            }
            else if((abs(idealH - H) < 0.1)){
                 step += 1;
                 Enes100.println("Reached idealH... ");
            }
            break; // ADDED BREAK
           
        case 2://move to mission
            Enes100.println(step);
            Enes100.println("Moving to Mission Site");
            // idealY is kept as curY until X is reached, which seems like a bug.
            // If the target is static, idealY should be a constant. Keeping your logic:
            if (abs(idealX-curX) > 0.1 || abs(idealY-curY) > 0.1){
                    if (starting_point == 'a')
                    {
                        idealY = 1.5;
                        idealH = (M_PI / 2.0);
                        Enes100.println("Moving to mission site B...");
                         translate(curX, curY, idealH, idealX, idealY);
                    }
                
                else if (starting_point == 'b')
                    {
                        idealY = 0.5;
                        idealH = -1 * (M_PI / 2.0);
                        Enes100.println("Moving to mission site A...");
                        translate(curX, curY, idealH, idealX, idealY);
                    }
            }
            else { // Reached target
                    step += 1;
                    break;
            }
            
        case 3: //make sure that the arduino is at the mission zone.
            stop();
            break;

        avoid();
    }
}
*/




//start the loop here
void loop(){
  /*left();
  delay(5000);
  right();
  delay(5000);
  stop();
  delay(500);*/
  float curX = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
  float curY = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
  float curH = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
    //translate(curX, curY, idealX, idealY);
  
  if (!Enes100.isVisible())
  {
    stop();
    Enes100.println("Waiting for aruco marker...");
    return;
  }
    
    switch(step){
      case 0:
        Enes100.println("Marker visible, starting orientation.");
        step = 1;
        
        if(curY > 1.0){
          zone = 'b';
        }else if(curY < 1.0){
          zone = 'a';
        } 
        else {
            //top left is b
            // For Arduino, avoid exit(); maybe print a warning
            Enes100.println("Warning: posy == 1.0, defaulting to 'a'");
            return 'a';
        }
        Enes100.print(zone);
        if(zone == 'a'){
          idealX = 0.35;
          idealY = 1.50;
        }else{
          idealX = 0.65;
          idealY = 0.5;
          idealH = (-M_PI/2) + 0.2;

        }
        break;
      case 1:
        Enes100.println("Orienting to heading... CurH:");
        Enes100.println(curH);
        Enes100.println(" idealH =");
        Enes100.println(idealH);
        orient(idealH);
        if (abs(idealH-curH) < 0.2)
        {
            Enes100.println("reached ideal theta");
            stop();
            step = 2;
            break;
        }
        break;
      case 2:
        
        relmotion(curH, 'y', (idealY-curY));
        Enes100.print("moving in the y-axis...");
        Enes100.print("ideal y : ");
        Enes100.println(idealY);
        delay(100);
        if (abs(idealY-curY) < 0.05){
          Enes100.print("reached ideal Y of ");
          Enes100.println(idealY);
          stop();
          step = 3;
          break;
        }
        break;
      case 3:
      if (abs(idealY-curY) > 0.05){
        step = 2; 
        return;
        Enes100.println("reorienting in Y direction...");
      }  
      relmotion(curH, 'x', (idealX-curX));
        Enes100.print("moving in the x-axis... ");
        Enes100.print("ideal x : ");
        Enes100.println(idealX);
        if (abs(idealX-curX) < 0.02){
          Enes100.print("reached ideal Y of ");
          Enes100.print(idealY);
          Enes100.print("with current X of ");
          Enes100.println(curX);
          stop();
          step = 4;
          break;
        }
        break;  
      case 4:
        stop(); 
        step = 5;
      case 5:
        Enes100.println("Begun mission sequence...");
        delay(100);
        waterLevel();
        
        colorTest();
        waterPump();

        
        step = 6;
        
      case 6:
        stop();
    }
      

   
}

/*
void translate(float curX, float curY, float idealX, float idealY){
    if (abs(idealX-curX) > 0.05){
      relmotion(curH, 'x', (idealX-curX));
      Enes100.println("moving in the x-axis translate");
      delay(100);
    }
    else{
      stop();
    }
    if (abs(idealY-curY) > 0.05){
      relmotion(curH, 'y', idealY-curY);
      Enes100.println("moving in the y-axis translate");
      delay(100);
    }
    else{
      stop();
    }
 
}
*/
void waterPump(){
  analogWrite(pumpPin, 255);      //uncomment this and set pump anywhere from 0-255
  delay(25000); ///MAKE SURE RIGHT AMOUNT
  analogWrite(pumpPin, 0);      //uncomment this and set pump anywhere from 0-255
  delay(100); }
void waterLevel() {
  long duration;
  float distance;  //in mm
  digitalWrite (trig,LOW);
  delayMicroseconds (2);
  digitalWrite (trig,HIGH);
  delayMicroseconds (5);
  digitalWrite (trig,LOW);
  duration = pulseIn (echo,HIGH);
  distance = duration * .1715;    //number calculated from changing time from microseconds to seconds then multiplying by speed of sounds and changing into mm and dividing by 2 to account for the 2 way distance
  float height = 88.28-distance;   //change to whatever value the sensor sees to the floor minus 9
  int water;

  if (height < 25) { //if the ultrasonic distance sensor sees the ball in front of it
    water=20;
 }
  else if (height < 35) { //if the ultrasonic distance sensor sees the ball in front of it
    water=30;
 }
 else { //if the ultrasonic distance sensor sees the ball in front of it
    water=40;
 }

  Enes100.println("Water Level: ");
  Enes100.println(water);
  Enes100.println (" mm");
  Enes100.println("Height: ");
  Enes100.println(height);
  Enes100.println("Distance: ");
  Enes100.println(distance);
  }
void colorTest() {
   //read the device temperature
  uint8_t temp = ams.readTemperature();
  ams.startMeasurement(); //begin a measurement
  bool rdy = false;
  while(!rdy){
    delay(5);
    rdy = ams.dataReady();
  }
  //read the values!
  ams.readRawValues(sensorValues);
  uint16_t maxVal = 0;
  int maxIndex = -1;
  const char* colors[6] = {"Violet", "Blue", "Green", "Yellow", "Orange", "Red"};

  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] > maxVal) {
     maxVal = sensorValues[i];
     maxIndex = i;
      }
  }
  double colorVal=sensorValues[AS726x_VIOLET]+sensorValues[AS726x_BLUE]+sensorValues[AS726x_GREEN]+sensorValues[AS726x_YELLOW]+sensorValues[AS726x_ORANGE]+sensorValues[AS726x_RED];
 if (colorVal<=300) {
  Enes100.println("Water is Clean!");
 }
 else if (colorVal>300) {
  Enes100.println("Water is Polluted!!");
 }
 //Serial.println(colorVal);

  }


void setMotor(int IN1, int IN2, int pwmPIN, int speed, bool positive) {
  digitalWrite(IN1, positive ? HIGH : LOW);
  digitalWrite(IN2, positive ? LOW : HIGH);
  analogWrite(pwmPIN, speed);
}
/*void back(){
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
    setMotor(A_IN1, A_IN2, A_ENA, SA*1.3, true);
    setMotor(B_IN3, B_IN4, B_ENB, SB*1.3, true);
    setMotor(C_IN5, C_IN6, C_ENC, SC*1.3, false);
    setMotor(D_IN7, D_IN8, D_END, SD*1.3, false);
}


void right(){
    setMotor(A_IN1, A_IN2, A_ENA, SA*1.3, false);
    setMotor(B_IN3, B_IN4, B_ENB, SB*1.3, false);
    setMotor(C_IN5, C_IN6, C_ENC, SC*1.3, true);
    setMotor(D_IN7, D_IN8, D_END, SD*1.3, true);
}
*/

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
void stop(){
  setMotor(A_IN1, A_IN2, A_ENA, 0, true);
  setMotor(B_IN3, B_IN4, B_ENB, 0, false);
  setMotor(C_IN5, C_IN6, C_ENC, 0, true);
  setMotor(D_IN7, D_IN8, D_END, 0, false);
}






void orient(float idealH) {
    Enes100.println("Starting PID Orientation...");
    
    // Reset PID variables
    float integral = 0;
    float prevError = 0;
    unsigned long lastTime = millis();
    
    // PID gains tuned for ~150ms delay system
    float Kp = 80;  // Proportional gain
    float Ki = 3.0;    // Integral gain
    float Kd = 20.0;   // Derivative gain
    
    // Speed constraints
    const int MIN_SPEED = 135;
    const int MAX_SPEED = 170;
    
    while (true) {
        float curH = Enes100.getTheta();
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        
        // Prevent division by zero on first iteration
        if (dt < 0.001) dt = 0.001;
        
        // Calculate shortest-path error
        float error = idealH - curH;
        
        // Wrap to [-PI, PI] for shortest rotation
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        
        // Check if we've arrived (tolerance: ~0.017 radians ≈ 1 degree)
        if (abs(error) < 0.1) {
            stop();
            Enes100.println("Orientation Complete!");
            Enes100.print("Final error (deg): ");
            Enes100.println(error * 180.0 / M_PI);
            break;
        }
        
        // PID calculations
        integral += error * dt;
        
        // Anti-windup: cap integral term
        float integralMax = 30.0;
        if (integral > integralMax) integral = integralMax;
        if (integral < -integralMax) integral = -integralMax;
        
        float derivative = (error - prevError) / dt;
        
        // Calculate PID output
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        
        // Determine rotation direction and speed
        bool isCCW = (output > 0);
        int speed = abs((int)output);
        
        // Apply speed limits
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        
        // Slow down as we approach target (within ~20 degrees)
        float absError = abs(error);
        if (absError < 0.35) {
            int scaledSpeed = MIN_SPEED + (int)((speed - MIN_SPEED) * (absError / 0.35));
            if (scaledSpeed < MIN_SPEED) scaledSpeed = MIN_SPEED;
            speed = scaledSpeed;
        }
        
        // Apply rotation with scaling factors for off-center mass
        if (isCCW) {
            // CCW rotation
            setMotor(A_IN1, A_IN2, A_ENA, speed, false);           // Front Right
            setMotor(B_IN3, B_IN4, B_ENB, speed, true);          // Back Right
            setMotor(C_IN5, C_IN6, C_ENC, speed * 0.81, false);    // Back Left
            setMotor(D_IN7, D_IN8, D_END, speed * 0.81, true);   // Front Left
        } else {
            // CW rotation
            setMotor(A_IN1, A_IN2, A_ENA, speed, true);          // Front Right
            setMotor(B_IN3, B_IN4, B_ENB, speed, false);           // Back Right
            setMotor(C_IN5, C_IN6, C_ENC, speed * 0.81, true);   // Back Left
            setMotor(D_IN7, D_IN8, D_END, speed * 0.81, false);    // Front Left
        }
          
        // Update for next iteration
        prevError = error;
        lastTime = now;
        
        // Small delay to prevent overwhelming the controller
        delay(5);
    }
}


// Modified relmotion to use heading-corrected straight motion
void relmotion(float heading, char axis, float delta){
  int headingIdx = -1, moveIdx = -1;

  if (abs(heading) < 0.3) headingIdx = 0;
  else if (abs(heading - 1.57) < 0.3) headingIdx = 1;
  else if (abs(abs(heading) - 3.14) < 0.3) headingIdx = 2;
  else if (abs(heading + 1.57) < 0.3) headingIdx = 3;
  else
  {
    Enes100.print("Reorienting to heading... CurH:");
        Enes100.print(heading);
        Enes100.print(" idealH =");
        Enes100.println(idealH);
        orient(idealH);
        if (abs(idealH-heading) < 0.2)
        {
            Enes100.println("reoriented to ideal theta");
            stop();
        }
  }
 
  if (axis == 'x') {
    if (delta > 0.1) moveIdx = 3;
    else if (delta < -0.1) moveIdx = 2;
  } else if (axis == 'y') {
    if (delta > 0.1) moveIdx = 0;
    else if (delta < -0.1) moveIdx = 1;
  }

  void (*moveRelative[4][4])(float targetHeading, int baseSpeed) = {
    { rightStraight, leftStraight, backStraight, goStraight },
    { goStraight, backStraight, leftStraight, rightStraight },
    { leftStraight, rightStraight, goStraight, backStraight },
    { backStraight, goStraight, rightStraight, leftStraight }
  };
/*
goStraight
backStraight
leftStraight
rightStraight
*/

   if (headingIdx != -1 && moveIdx != -1) {
    moveRelative[headingIdx][moveIdx](heading, SPEED);
   }


}


// Dynamic straight-line motion with heading correction
void goStraight(float targetHeading, int baseSpeed) {
    unsigned long now = millis();
    float dt = (now - lastStraightTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
   
    float curH = Enes100.getTheta();
   
    // Calculate heading error (how far we've rotated from target)
    float headingError = targetHeading - curH;
   
    // Wrap to [-PI, PI]
    while (headingError > M_PI) headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;
   
    // Calculate derivative
    float derivative = (headingError - prevHeadingError) / dt;
   
    // PD correction (proportional + derivative)
    float correction = (straightKp * headingError) + (straightKd * derivative);
   
    // Apply correction by adjusting left/right wheel speeds
    // Positive error means we've rotated CW, need to speed up right side
    int leftSpeed = baseSpeed - (int)correction;
    int rightSpeed = baseSpeed + (int)correction;
   
    // Clamp speeds to valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
   
    // Apply differential speeds (with your existing scaling factors)
    // Right side: Motors A (front) and B (back)
    setMotor(A_IN1, A_IN2, A_ENA, rightSpeed, !false);
    setMotor(B_IN3, B_IN4, B_ENB, rightSpeed, !true);
   
    // Left side: Motors C (back) and D (front) with 0.75 scaling
    setMotor(C_IN5, C_IN6, C_ENC, leftSpeed, true);
    setMotor(D_IN7, D_IN8, D_END, leftSpeed, !false);
   
    // Update for next iteration
    prevHeadingError = headingError;
    lastStraightTime = now;
}

// Similar function for backward motion
void backStraight(float targetHeading, int baseSpeed) {
    unsigned long now = millis();
    float dt = (now - lastStraightTime) / 1000.0;
    if (dt < 0.005) dt = 0.005;
   
    float curH = Enes100.getTheta();
    float headingError = targetHeading - curH;
   
    while (headingError > M_PI) headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;
   
    float derivative = (headingError - prevHeadingError) / dt;
    float correction = (straightKp * headingError) + (straightKd * derivative);
   
    // For backward motion, correction logic is inverted
    int leftSpeed = baseSpeed + (int)correction;
    int rightSpeed = baseSpeed - (int)correction;
   
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
   
    // Backward motion (directions reversed from forward)
    setMotor(A_IN1, A_IN2, A_ENA, rightSpeed, !true);
    setMotor(B_IN3, B_IN4, B_ENB, rightSpeed, !false);
    setMotor(C_IN5, C_IN6, C_ENC, leftSpeed, false);
    setMotor(D_IN7, D_IN8, D_END, leftSpeed, !true);
   
    prevHeadingError = headingError;
    lastStraightTime = now;
}



void leftStraight(float targetHeading, int baseSpeed) {
    unsigned long now = millis();
    float dt = (now - lastStraightTime) / 1000.0;
    if (dt < 0.005) dt = 0.005;
   
    float curH = Enes100.getTheta();
   
    // Calculate heading error (how far we've rotated from target)
    float headingError = targetHeading - curH;
   
    // Wrap to [-PI, PI]
    while (headingError > M_PI) headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;
   
    // Calculate derivative
    float derivative = (headingError - prevHeadingError) / dt;
   
    // PD correction (proportional + derivative)
    float correction = (straightKp * headingError) + (straightKd * derivative);
   
    // Apply correction by adjusting left/right wheel speeds
    // Positive error means we've rotated CW, need to speed up right side
    int leftSpeed = baseSpeed - (int)correction;
    int rightSpeed = baseSpeed + (int)correction;
   
    // Clamp speeds to valid range
    leftSpeed = constrain(leftSpeed, 50, 255);
    rightSpeed = constrain(rightSpeed, 50, 255);
   
    // Apply differential speeds (with your existing scaling factors)
    // Right side: Motors A (front) and B (back)
    setMotor(A_IN1, A_IN2, A_ENA, 1.2*rightSpeed, !true);
    setMotor(B_IN3, B_IN4, B_ENB, 1.2*rightSpeed, !true);
   
    // Left side: Motors C (back) and D (front) with 0.75 scaling
    setMotor(C_IN5, C_IN6, C_ENC, 1.2*leftSpeed, false);
    setMotor(D_IN7, D_IN8, D_END, 1.2*leftSpeed, !false);
   
    // Update for next iteration
    prevHeadingError = headingError;
    lastStraightTime = now;
}

void rightStraight(float targetHeading, int baseSpeed) {
    unsigned long now = millis();
    float dt = (now - lastStraightTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
   
    float curH = Enes100.getTheta();
   
    // Calculate heading error (how far we've rotated from target)
    float headingError = targetHeading - curH;
   
    // Wrap to [-PI, PI]
    while (headingError > M_PI) headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;
   
    // Calculate derivative
    float derivative = (headingError - prevHeadingError) / dt;
   
    // PD correction (proportional + derivative)
    float correction = (straightKp * headingError) + (straightKd * derivative);
   
    // Apply correction by adjusting left/right wheel speeds
    // Positive error means we've rotated CW, need to speed up right side
    int leftSpeed = baseSpeed - (int)correction;
    int rightSpeed = baseSpeed + (int)correction;
   
    // Clamp speeds to valid range
    leftSpeed = constrain(leftSpeed, 50, 255);
    rightSpeed = constrain(rightSpeed, 50, 255);
   
    // Apply differential speeds (with your existing scaling factors)
    // Right side: Motors A (front) and B (back)
    setMotor(A_IN1, A_IN2, A_ENA, rightSpeed, !false);
    setMotor(B_IN3, B_IN4, B_ENB, rightSpeed, !false);
   
    // Left side: Motors C (back) and D (front) with 0.75 scaling
    setMotor(C_IN5, C_IN6, C_ENC, leftSpeed, true);
    setMotor(D_IN7, D_IN8, D_END, leftSpeed, !true);
   
    // Update for next iteration
    prevHeadingError = headingError;
    lastStraightTime = now;
}

