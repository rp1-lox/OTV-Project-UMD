#ifndef WATERBOYS_H
#define WATERBOYS_H
/*
for global variables label extern in header file and define only once in any of the files 
e.g extern float heading; and float heading = ....;

*/

extern float curX;
extern float curY;

extern float heading;

extern float X;
extern float Y;
extern float H;
extern bool V;

extern int step;
extern int m_step;
extern char starting_point;

extern float idealH;
extern float idealX;
extern float idealY;

extern int SA;
extern int SB;
extern int SC;
extern int SD;

// Motor A (HB1)(OUT1/OUT2)(Front Right)
extern const int A_IN1 = 22;
extern const int A_IN2 = 23;
extern const int A_ENA = 4;   // PWM

// Motor B (HB1)(OUT3/OUT4)(Back Right)
extern const int B_IN3 = 24;
extern const int B_IN4 = 25;
extern const int B_ENB = 5;   // PWM

// Motor C (HB2)(OUT1/OUT2)(Back Left)
extern const int C_IN5 = 26;
extern const int C_IN6 = 27;
extern const int C_ENC = 6;   // PWM

// Motor D (HB2)(OUT3/OUT4) (Front Left)
extern const int D_IN7 = 28;
extern const int D_IN8 = 29;
extern const int D_END = 7;   // PWM

const int SPEED = 200; // Change this value as needed(changes the pwm)

extern int SA = (0.9*SPEED);
extern int SB = (0.9*SPEED);
extern int SC = (0.9*SPEED);
extern int SD = (0.9*SPEED);


/*
for functions write function declaration here and include "waterboys.h" in all files
*/

void go();
void back();
void left();
void right();
void ccw();
void cw();
void stop();

void relmotion(float heading, char axis, float delta);

void setMotor(int IN1, int IN2, int pwmPIN, int speed, bool positive);

void avoid();

void orient(float H, float t);

void translate(float curX, float curY, float idealX, float idealY);

char a_or_b(float posy);




#endif 