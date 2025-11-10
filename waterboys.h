#ifdef WATERBOYS_H
#define WATERBOYS_H

extern float X;//x pos
extern float Y;//y pos
extern float H;//heading
extern bool V;//is visible
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

char a_or_b (float posx, float posy);

bool isVisible();

void avoid()




#endif 

