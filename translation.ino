/*
There are a couple of things that need to be input into each function first:
    1. power
        - use the equations that we got in class today but we might need to do some
          more math by finding the resistance?? not so sure this might be done better with testing 
        - lets use the variable pwm as the power.
            - this should be an int between 0-255 but chat gpt said something about scaling it down
              between -1, 0
    2. pin #
    3. to get negative spin use the other pin in the h-bridge


*/

void go();
void back();
void left();
void right();

void go(){
    /*
    +i +j
    +j +i
    */
}

void back(){
    /*
    -i -j
    -j -i
    */
}

//etc 