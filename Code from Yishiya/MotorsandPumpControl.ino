const int pumpPin = 10; 
// Motor A (HB1)(OUT1/OUT2)
const int IN1 = 22;
const int IN2 = 23;
const int ENA = 4;   // PWM


// Motor B (HB1)(OUT3/OUT4)
const int IN3 = 24;
const int IN4 = 25;
const int ENB = 5;   // PWM


// Motor C (HB2)(OUT1/OUT2)
const int IN5 = 26;
const int IN6 = 27;
const int ENC = 6;   // PWM


// Motor D (HB2)(OUT3/OUT4)
const int IN7 = 28;
const int IN8 = 29;
const int END = 7;   // PWM


void setup() {


 pinMode(pumpPin, OUTPUT);
 analogWrite(pumpPin, 0); //off
  pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(ENA, OUTPUT);


 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(ENB, OUTPUT);


 pinMode(IN5, OUTPUT);
 pinMode(IN6, OUTPUT);
 pinMode(ENC, OUTPUT);


 pinMode(IN7, OUTPUT);
 pinMode(IN8, OUTPUT);
 pinMode(END, OUTPUT);


 motorsOff();


}


void loop() {
//analogWrite(pumpPin, 255);      //uncomment this and set pump anywhere from 0-255
motorsForwards(200);
delay(3000);
motorsBackwards();
delay(3000);
motorsOff();
delay(3000);


}
void motorsOff() {
 // Option 1: “coast” stop (both inputs LOW)
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, LOW);
 digitalWrite(IN5, LOW);
 digitalWrite(IN6, LOW);
 digitalWrite(IN7, LOW);
 digitalWrite(IN8, LOW);
 analogWrite(ENA, 0);
 analogWrite(ENB, 0);
 analogWrite(ENC, 0);
 analogWrite(END, 0);
}
void motorsForwards(int speedIn) {
 // Option 1: “coast” stop (both inputs LOW)
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 digitalWrite(IN5, HIGH);
 digitalWrite(IN6, LOW);
 digitalWrite(IN7, HIGH);
 digitalWrite(IN8, LOW);
 analogWrite(ENA, speedIn);
 analogWrite(ENB, speedIn);
 analogWrite(ENC, speedIn);
 analogWrite(END, speedIn);
}
void motorsBackwards() {
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, HIGH);
 digitalWrite(IN5, LOW);
 digitalWrite(IN6, HIGH);
 digitalWrite(IN7, LOW);
 digitalWrite(IN8, HIGH);
 analogWrite(ENA, 200);
 analogWrite(ENB, 200);
 analogWrite(ENC, 200);
 analogWrite(END, 200);
}
void motor1Forwards() {
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 analogWrite(ENA, 200);
}
void motor1Backwards() {
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 analogWrite(ENA, 200);
}
void motor2Forwards() {
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 analogWrite(ENB, 200);
}
void motor2Backwards() {
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, HIGH);
 analogWrite(ENB, 200);
}
void motor3Forwards() {
 digitalWrite(IN5, HIGH);
 digitalWrite(IN6, LOW);
 analogWrite(ENC, 200);
}
void motor3Backwards() {
 digitalWrite(IN5, LOW);
 digitalWrite(IN6, HIGH);
 analogWrite(ENC, 200);
}
void motor4Forwards() {
 digitalWrite(IN7, HIGH);
 digitalWrite(IN8, LOW);
 analogWrite(END, 200);
}
void motor4Backwards() {
 digitalWrite(IN7, LOW);
 digitalWrite(IN8, HIGH);
 analogWrite(END, 200);
}

