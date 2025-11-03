#include <stdio.h>
#include <translation.ino>
#include <obstacleavoid.ino>
#include <move_to_mission.ino>
#include <wifimodule.h>
#include <enes100.h>
int program = 0;
//this is an exceptionally bad way to do this but we're on a crunch. too bad!
// program 0 = locomotion program - also doubles as RX since it relies on the getx and gety
// program 1 = turning 
// program 2 = tx
// program 3 = obstacle avoidance
// program 4 = mission demo
void main(){
    if(program == 0){
            idealx = 0.5;
            idealy = 1.5;
            heading = 0;
            translation.loop();
            idealx = 3.4;
            idealy = 1.5;
            heading = 90;
            translation.loop();
    }else if(program == 1){
        idealx = Enes100.getx();
        idealy = Enes100.getY();
        heading = 0;
        translation.loop();
        heading = 90;
        translation.loop();
        heading = 180(){
        translation.loop();
        }
    }else if(program == 2){
// already demonstrated no need to exist
    }else if(program == 3){
        idealx = Enes100.getx() + 2;
        idealy = Enes.gety();
        heading = 90;
        translation.loop();
    }else if(program ==4){
// relies on yeshiah
    }
}