#include <studio.h>
#include "waterboys.h"
#include "Enee100.h"

void setup(){
    /*
    initialize all pin numbers here
    
    
    
    
    
    
    */
}


//define all extern global variables here
float X;//x pos
float Y;//y pos
float H;//heading
bool V;//visibility

//define global variables here
int step = 0;
int m_step = 0;
char starting_point = 0;


//start the loop here
void loop(){
    X = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
    Y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
    H = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
    V = Enes100.isVisible(); // Is your aruco visible? True or False.
    if( V == 0 ){
        Enes100.println("OTV not visible");
        return;
    }
    switch(step){
        case 1://determine the starting point
            starting_point = a_or_b (Y);
            stept += 1;    
        case 2://move to mission
            if(starting_point == 'a'){
                if (abs(/*idealX-curX*/) > 0.1 || abs(/*idealY-curY*/) > 0.1){
                    translate(/*float curX, float curY, float idealX, float idealY*/);
                }
                else if (abs(/*idealX-curX*/) > 0.1 && abs(/*idealY-curY*/) < 0.1){
                    step += 1;
                    break;
                }
            }
            else if(starting_point == 'b'){
                if (abs(/*idealX-curX*/) > 0.1 || abs(/*idealY-curY*/) > 0.1){
                    translate(/*float curX, float curY, float idealX, float idealY*/);
                }
                else if (abs(/*idealX-curX*/) > 0.1 && abs(/*idealY-curY*/) < 0.1){
                    step += 1;
                    break;
                }
            }
            else{
                Enes100.println("starting point could not be determined");
                step = 0;//go back and determine the starting point
                break;
            }
        case 3:

            







    
    
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