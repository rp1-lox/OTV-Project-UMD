#include <translation.ino>
int step = 0;
int m_step = 0;
char starting point;
void loop(
    float x = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
    float y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
    float heading = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
    bool v = Enes100.isVisible(); // Is your aruco visible? True or False.

    if (step == 0){
        if(/*if in starting point A*/){
            
        }
        else if()

    }
)

/* Procedure
****can only have one loop() function so we will need to use a traking system and**** 
****if statements to track progress                                              ****
outside of the loop()
    1. Grab heading and coordinates to determine where we are and where we need to go
    2.Determine if starding zone is A or B
    3.drive to preset coordinates depending on (2.)
    4.run mission things
    5.orient to 0 degrees
    6.run obstacle avoidance
    7. nav under log and into endzone












*/