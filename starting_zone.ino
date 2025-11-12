#include <stdio.h>
#include <stlib.h>
#include "waterboys.h"

char a_or_b (float posy){
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