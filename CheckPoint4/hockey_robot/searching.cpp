#include "searching.h"
#include "motor_motion.h"

const unsigned long sample_us = 100000; 

void search_sensor_init()
{
  pinMode(light_sensor, INPUT);
  pinMode(IR_sensor, INPUT);
}

int read_light_sensor(int sample_times)
{
  int v = 0;
  for (int i = 0 ; i < sample_times; i++)
    v += analogRead(light_sensor);
  
  return v / sample_times;
}

bool try_search_puck()
{
  stop_motors();
  delay(50);

  const long ticks_360 = (long)(360.0f * PPA); 
  const int sample_every_ticks = 50;          

  reset_ticks();
  long last_sample_tick = 0;

  int   best_light  = 1023;
  float best_angle  = 0.0f;

  int pwm = 100;         
  rotate_cw(pwm);        

  while (true) {
    long d = labs(get_diff_ticks());

    if (d >= ticks_360) {
      break;
    }

    if (d - last_sample_tick >= sample_every_ticks) {
      last_sample_tick = d;

      int v = read_light_sensor(3);
      float angle = 360.0f * (float)d / (float)ticks_360;

      if (v < best_light) {
        best_light = v;
        best_angle = angle;
      }
    }

    delay(2);
  }

  stop_motors();

  if (best_light > detected_puck_val) {
    return false;
  }

  float delta = best_angle - 360.0f;   

  rotate_angle(delta);     
  stop_motors();
  reset_ticks();           

  return true;
}

float measure_ratio_once(int pin)
 {
  unsigned long count_low = 0;
  unsigned long total_count = 0;

  unsigned long t_end = micros() + sample_us;
  while ((long)(t_end - micros()) > 0) {
    if (digitalRead(pin) == LOW)
      count_low++;
    total_count++;
  }

  if (total_count == 0) return -1;
  return (float)count_low / (float)total_count;
}

float measure_beacon_ratio(int pin, float duration_seconds)
 {
  unsigned long duration_ms = (unsigned long)(duration_seconds * 1000);
  unsigned long start = millis();

  float rmin = 999;
  float rmax = -1;
  float sum  = 0;
  int count  = 0;

  while (millis() - start < duration_ms) {
    float ratio = measure_ratio_once(pin);

    if (ratio >= 0) {
      if (ratio < rmin) rmin = ratio;
      if (ratio > rmax) rmax = ratio;
      sum += ratio;
      count++;
    }

    delay(20);
  }

  if (count == 0)
    return -1;

  float avg = sum / count;

  return avg;
}


BeaconType detect_beacon()
{
  float ratio = measure_beacon_ratio(IR_sensor, 1.0); 

  if (ratio >= 0.20 && ratio <= 0.32) 
    return B600;

  if (ratio >= 0.40 && ratio <= 0.45) 
    return B1500;

  return None;
}

bool try_search_goal()
{
  stop_motors();
  delay(50);

  const int   steps = 12;                    
  const float angle_step = 360.0f / steps;        

  int   best_step = -1;      
  float best_err  = 1e9;     

  float ideal_ratio = 0.0f;
  float max_toler   = 0.0f;  

  if (target_beacon == B600) {
    ideal_ratio = 0.25f;   
    max_toler   = 0.05f;     
  } else if (target_beacon == B1500) {
    ideal_ratio = 0.425f;   
    max_toler   = 0.05f;     
  } else {
    return false;
  }

  for (int i = 0; i < steps; i++) {

    stop_motors();
    delay(50);   

    float ratio = measure_beacon_ratio(IR_sensor, 0.2f);  

    if (ratio >= 0.0f) {
      float err = fabs(ratio - ideal_ratio);  

      if (err < best_err && err <= max_toler) {
        best_err  = err;
        best_step = i;
      }
    }

    rotate_angle(angle_step);
    delay(20);
  }

  stop_motors();

  if (best_step < 0) {
    return false;
  }

  float target_angle = best_step * angle_step;   
  float delta = target_angle - 360.0f;          

  if (delta > 180.0f)  delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;

  rotate_angle(delta);
  stop_motors();
  reset_ticks();   

  return true;
}


