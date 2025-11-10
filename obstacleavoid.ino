#include <stdio.h>
#include <translation.ino>
#include <ultrasonic.ino>
#include <wifimodule.h>
#include <translation.ino>

using namespace std;

float position == wifimodule.getY();
int samples == 9;
int depths[samples];
float tolerances = 1; //this should be replaced by something else!!!! determine proper tolerances!!!!!!!
float blockPos[] = {0.5, 1, 1.5};
float score[3];
int chosenBlock = 0;


void avoid(){
for (int i = 0; i < samples; i++){
    idealy = 0.25+(((i*0.5)/(samples/3)));
    translation.loop();
    depth[i] = ultrasonic.depth();
}


for(int i = 0; i < samples; i++){
    if((i-3 <= 0) && (depth[i] <= tolerance)){
        score[0] += 1;
    }else if(i-6 <= 0){
        score[1] += 1;
    }else if (depth[i] <= tolerance){
        score[2] += 1;
    }
}
for(int i = 0; i < samples; i++){
    if(score[i]==2){
        chosenBlock = i;
        break;
    }
}
idealy = blockPos[chosenBlock];
translation.loop();
idealx = 1.9;
translation.loop();
}



//float x = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
//float y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
//loat heading = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
/*
//need distance from OS sensor and 

//void avoid(){
//    if (!Enes100.isVisible()) {
//        stop();
// send message that the vision system is not working 
//    }
//    else if(x < 2.8){
//        if(delta() < ){
//            go(); 
       }
        else if (delta() > ){
            if ( y > 1){
                right();
          }
            else if (y < 1){
                left();
            }
        }
    }
}
*/

