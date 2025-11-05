#include <stdio.h>
#include <translation.ino>
#include <ultrasonic.ino>
#include <wifimodule.h>
#include <translation.ino>


float x = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
float y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
float heading = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.

//need distance from OS sensor and 

void loop(){
    float posX = Enes100.getX();
    float posY = Enes100.getY();
    if (!Enes100.isVisible()) {
        stop();
// send message that the vision system is not working 
    }
    else if(x < 2.8){
        if(delta() < /*tolerence*/){
            go()
        }
        else if (delta() > /*tolerence*/){
            if ( y > 1){
                right()
            }
            else if (y < 1){
                left()
            }
        }
    }
}

