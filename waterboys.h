#ifdef WATERBOYS_H
#define WATERBOYS_H
/*
for global variables label extern in header file and define only once in any of the files 
e.g extern float heading; and float heading = ....;

*/

/*
for functions write function declaration here and include "waterboys.h" in all files
*/

void go();
void back();
void left();
void right();
void ccw();
void cw();
void stop();

void relmotion(float heading, char axis, float delta);

char a_or_b (float posy);

bool isVisible();

void avoid();

void ortient(float H, float t);

void translate(float curX, float curY, float idealX, float idealY, float H, float deltaX, float deltaY);




#endif 

