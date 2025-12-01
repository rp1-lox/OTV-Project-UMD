#include <stdio.h>
#include "waterboys.h"
#include "Enes100.h"

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

    // Initialize Enes100 Library
    // Team Name, Mission Type, Marker ID, Room Number, Wifi Module TX Pin, Wifi Module RX Pin
    int txpin = 50;
    int rxpin = 51;
    Enes100.begin("Waterboys", WATER, 284, 1120, txpin, rxpin);
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
int step = 1;
int m_step = 0;
char starting_point = 0;


//start the loop here
void loop(){
    float curX = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
    float curY = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
    float H = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.

    float idealH = 0; ///////weird
    float idealX = 0;
    float idealY = 0;

    //if( V == 0 ){
     //   Enes100.println("OTV not visible");
      //  de(100);
       // return;
    //}
    switch(step){
        case 1://determine the starting point
            starting_point = a_or_b(curY);
            step += 1;
            Enes100.print("Starting point: ");
            Enes100.println(starting_point);
        case 2://rotate to propper orientation
            if(starting_point == 'a'){ ////////
                if(abs(idealH-H) >= 0.1){
                    orient(H, idealH/*set the idealH*/);
                     Enes100.println(step);
                     step+=1; 
                     return;
                }
                else if((abs(idealH-H) < 0.1)){
                    step += 1;
                    break;
                }
                else{
                   Enes100.println("case 2, 'a', could rotate to idealH/*change this*/"); 
                   return;//check if this sends back to the loop()
                   Enes100.println("if you're reading this we need orient");
                }
            }
            else if(starting_point == 'b'){
                if(abs(idealH-H) >= 0.1){
                    //orient(H, idealH/*set the idealH*/);
                }
                else if((abs(idealH-H) < 0.1)){
                    step += 1;
                    break;
                }
                else{
                   Enes100.println("case 2, 'b', could rotate to idealH/*change this*/"); 
                   return;//check if this sends back to the loop()
                }
            }
            else{

            }
        case 3://move to mission
            Enes100.println(step);
            idealX = 0.5;
            idealY = curY;
            Enes100.println(idealX);
            Enes100.println(idealY);
            Enes100.println(curX);
            Enes100.println(curY);
            
            if(starting_point == 'a'){
                if (abs(idealX-curX) > 0.1 || abs(idealY-curY) > 0.1){
                    translate(curX, curY, idealX, idealY);
                }
                else if (abs(idealX-curX) > 0.1 && abs(idealY-curY) < 0.1){
                    step += 1;
                    break;
                }
            }
            else if(starting_point == 'b'){
                if ((abs(idealX-curX) > 0.1) || (abs(idealY-curY) > 0.1)){
                    translate(curX, curY, idealX, idealY);
                }
                else if ((abs(idealX-curX) > 0.1 && (abs(idealY-curY) < 0.1))){
                    step += 1;
                    break;
                }
            }
            else{
                Enes100.println("case 3: starting point could not be determined");
                step = 0;//go back and determine the starting point
                break;
            }
        case 4: //make sure that the arduino is at the mission zone. do this by using the 
          break;







    
    
    }
}

/* Procedure
****can only have one loop() function so we will need to use a traking system and**** 
****if statements to track progress                                              ****
outside of the loop()
    1.Grab heading and coordinates to determine where we are and where we need to go
    2.Determine if starding zone is A or B
    3.drive to preset coordinates depending on (2.)
    4.run mission things
    5.orient to 0 degrees
    6.run obstacle avoidance
    7. nav under log and into endzone
*/