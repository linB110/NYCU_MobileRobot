#include "obstacle_avoidance.h"

#define EI_NOTEXTERNAL  
#include <EnableInterrupt.h>

volatile bool right_hit = false;
volatile bool left_hit  = false;
volatile bool end_hit   = false;

void touch_sensor_init()
{
  pinMode(touch_sensor_right, INPUT);
  pinMode(touch_sensor_left, INPUT);
  pinMode(touch_sensor_endmove, INPUT);
  
  enableInterrupt(touch_sensor_right, touched_right, FALLING);
  enableInterrupt(touch_sensor_left, touched_left, FALLING);
  enableInterrupt(touch_sensor_endmove, touched_target, FALLING);
}

void touched_right()
{
  right_hit = true;
}

void touched_left()
{
  left_hit = true;
}

void touched_target()
{
  end_hit = true;
}
