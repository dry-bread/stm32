#include <math.h>
#include "can.h"


float Vy=1,Vx=1,angularVell=1;//设置一下基础速度


typedef struct
{
    float v1;
    float v2;
    float v3;
}ActThreeVell;


ActThreeVell ThreeWheelVellControl2(float Vx, float Vy, float angularVell)
{
#define AFA 60
#define L   2

ActThreeVell vell;
float theta = 0;

vell.v1 = (float)(-cos((AFA + theta) / 180.0f*3.1415926f) * Vx - sin((theta + AFA) / 180.0f*3.1415926f) * Vy + L * angularVell);

vell.v2 = (float)(cos(theta / 180.0f*3.1415926f) * Vx + sin(theta /180.0f*3.1415926f) * Vy      + L * angularVell);

vell.v3 = (float)(-cos((AFA - theta) / 180.0f * 3.1415926f) * Vx + sin((AFA - theta) / 180.0f*3.1415926f) * Vy + L * angularVell);

return vell;





}


typedef union{
        char ch[8];
        char ui8[8];
        uint16_t ui16[4];
        int in[2];
        float fl[2];
        double df;
}can_change_msg;





void send_wheel_msg(ActThreeVell vell)
{
  can_change_msg can_msg;
  

  can_msg.in[0]=1;
  can_msg.in[1]=(int)vell.v1;
  can_send(0,1,can_msg.ui8,8);
  can_msg.in[1]=(int)vell.v2;
  can_send(0,2,can_msg.ui8,8);
  can_msg.in[1]=(int)vell.v3;
  can_send(0,3,can_msg.ui8,8);
  


}



void forward()
{
  ThreeWheelVellControl2(0, Vy, 0);
}

void backward()
{
  ThreeWheelVellControl2(0, 0-Vy,0);
}

void left_translation()
{
  ThreeWheelVellControl2(0-Vx,0,0);
}

void right_translation()
{
  ThreeWheelVellControl2(Vx, 0,0);
}

void turn_left()
{
  ThreeWheelVellControl2(0,0, angularVell);
}

void turn_right()
{
  ThreeWheelVellControl2(0,0, 0-angularVell);
}
