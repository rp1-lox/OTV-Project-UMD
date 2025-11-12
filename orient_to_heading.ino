#include <stdio.h>
#include "Enes100.h"
#include "waterboys.h"

//1. check the orientation of the vehicle using heading getTheta()
//2. rotate until headingID is north
//
int headingIdx = 5;
float heading = Enes100.getTheta();


if (heading > 0) && (heading < 3.14)
{

}, //rotate clockwise for (function related to heading value) seconds
// if heading is between 0 and -3.14, rotate counter clockwise for (function related to heading value) seconds
// check if we are facing north facing north
// if (abs(heading) < 0.2) headingIdx = 0;
// else loop
void ortient(float H, float t){// H is the current heading and t is the desired heading
    if(abs(H-h) > .2){
        ccw();
    }
}