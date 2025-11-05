#include <translation.ino>
void loop(
    float x = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
    float y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
    float heading = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
    int step = 0;
)

/* Procedure
****can only have one loop() function so we will need to use a traking system and**** 
****if statements to track progress                                              ****
    1. Grab heading and coordinates to determine where we are and where we need to go 
    2. invoke translate function
    3. 











*/