#include <Enes100.h>
#include<math.h>
void translate(float curX, float curY, float idealX, float idealY);
void relmotion(float heading, char axis, float delta);

// Motor A (HB1)(OUT1/OUT2)(Front Right)
int A_IN1 = 22;
int A_IN2 = 23;
int A_ENA = 4;   // PWM

// Motor B (HB1)(OUT3/OUT4)(Back Right)
int B_IN3 = 24;
int B_IN4 = 25;
int B_ENB = 5;   // PWM

// Motor C (HB2)(OUT1/OUT2)(Back Left)
int C_IN5 = 26;
int C_IN6 = 27;
int C_ENC = 6;   // PWM

// Motor D (HB2)(OUT3/OUT4) (Front Left)
int D_IN7 = 28;
int D_IN8 = 29;
int D_END = 7;   // PWM

const int SPEED = 200; // Change this value as needed(changes the pwm)

int SA = (1*SPEED);
int SB = (1*SPEED);
int SC = (0.75*SPEED);
int SD = (0.75*SPEED);

float Kp = 150.0; // Proportional gain (Power per radian of error)
float Ki = 0.5;   // Integral gain (Fixes small steady-state errors)
float Kd = 10.0;  // Derivative gain (Dampens oscillation)

float prevError = 0;
float integral = 0;
unsigned long lastTime = 0;

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

//for wifi module
//test
    // Initialize Enes100 Library
    // Team Name, Mission Type, Marker ID, Room Number, Wifi Module TX Pin, Wifi Module RX Pin
    int txpin = 50;
    int rxpin = 51;
    Enes100.begin("Waterboys", WATER, 284, 1201, txpin, rxpin);
    // At this point we know we are connected.
    Enes100.println("Connected...");

    float V = Enes100.isVisible(); // Is your aruco visible? True or False.
    
}


//define all extern global variables here
float X;//x pos
float Y;//y pos
float H;//heading
bool V;//visibility

//define global variables here
 
float idealX = 3.5;
float idealY = 3.5;
float idealH = M_PI/2;
int step = 0;


//start the loop here
void loop() {
    float curX = Enes100.getX();
    float curY = Enes100.getY();
    float curH = Enes100.getTheta();  // Get once
    translate(curX, curY, curH, idealX, idealY);  // Pass it in
    orient(M_PI/2);
    /*switch(step) {
      case 0:
        // Add a step 0 to handle orientation just once
        orient(idealH); 
        step++; 
        break;

      case 1:
        // Now that we are oriented, move Y
        relmotion(curH, 'y', (idealY - curY)); // Use curH here, not H (which was undefined global)
        Enes100.println("moving in the y-axis");
        if (abs(idealY - curY) < 0.1) { // Increased tolerance slightly
             stop();
             step++;
        }
        break;

      case 2:
        relmotion(curH, 'x', (idealX - curX));
        Enes100.println("moving in the x-axis");
        if (abs(idealX - curX) < 0.1) {
             stop();
             step++;
        }
        break; 

      case 3:
        stop();
        Enes100.println("Mission Complete");
        break;
    }*/
}

void translate(float curX, float curY, float curH, float idealX, float idealY){
    float errorX = idealX - curX;
    float errorY = idealY - curY;
    
    // Priority 1: Fix X first
    if (abs(errorX) > 0.05) {
        relmotion(Enes100.getTheta(), 'x', errorX);
        Enes100.println("moving in the x-axis");
    }
    // Priority 2: Once X is good, fix Y
    else if (abs(errorY) > 0.05) {
        relmotion(Enes100.getTheta(), 'y', errorY);
        Enes100.println("moving in the y-axis");
    }
    // Both axes within tolerance
    else {
        stop();
        Enes100.println("Position reached!");
    }
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
void stop(){
  setMotor(A_IN1, A_IN2, A_ENA, 0, true);
  setMotor(B_IN3, B_IN4, B_ENB, 0, false);
  setMotor(C_IN5, C_IN6, C_ENC, 0, true);
  setMotor(D_IN7, D_IN8, D_END, 0, false);
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
  }
}

void orient(float idealH) {
    Enes100.println("Starting PID Orientation...");
    
    // Reset PID variables
    float integral = 0;
    float prevError = 0;
    unsigned long lastTime = millis();
    
    // PID gains tuned for ~150ms delay system
    float Kp = 80.0;  // Proportional gain
    float Ki = 3.0;    // Integral gain
    float Kd = 25.0;   // Derivative gain
    
    // Speed constraints
    const int MIN_SPEED = 60;
    const int MAX_SPEED = 200;
    
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
        if (abs(error) < 0.017) {
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
            setMotor(A_IN1, A_IN2, A_ENA, speed, true);           // Front Right
            setMotor(B_IN3, B_IN4, B_ENB, speed, false);          // Back Right
            setMotor(C_IN5, C_IN6, C_ENC, speed * 0.81, true);    // Back Left
            setMotor(D_IN7, D_IN8, D_END, speed * 0.81, false);   // Front Left
        } else {
            // CW rotation
            setMotor(A_IN1, A_IN2, A_ENA, speed, false);          // Front Right
            setMotor(B_IN3, B_IN4, B_ENB, speed, true);           // Back Right
            setMotor(C_IN5, C_IN6, C_ENC, speed * 0.81, false);   // Back Left
            setMotor(D_IN7, D_IN8, D_END, speed * 0.81, true);    // Front Left
        }
        
        // Update for next iteration
        prevError = error;
        lastTime = now;
        
        // Small delay to prevent overwhelming the controller
        delay(20);
    }
}
