#include <stdio.h>
#include <stlib.h>
#include <translation.ino>
#include <obstacleavoid.ino>
#include <move_to_mission.ino>
#include <wifimodule.ino>

char a_or_b (float posx, float posy){
    if(posY > 1.0){
        return 'a';
    }
    else if(posY < 1.0){
        return 'b';
    }
    else{
        exit(0);
    }
}