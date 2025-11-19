#include "waterboys.h"
#include <stdlib.h>  // optional, only if using exit()

char a_or_b(float posy) {
    if(posy > 1.0) return 'a';
    else if(posy < 1.0) return 'b';
    else {
        //top left is b
        // For Arduino, avoid exit(); maybe print a warning
        Serial.println("Warning: posy == 1.0, defaulting to 'a'");
        return 'a';
    }
}
