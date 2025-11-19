#include "searching.h"
#include "motor_motion.h"

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
  delay(5);

  int lightest = 1023;      
  int lightest_step = 0;

  const float angle_step = 360.0f / Search_step; 

  for (int i = 0; i < Search_step; i++) {
    int v = read_light_sensor(5);

    if (v < lightest) {
      lightest = v;
      lightest_step = i;
    }

    delay(5);
    rotate_angle(angle_step);
    delay(5);
  }

  if (lightest > detected_puck_val) {
    return false;
  }

  if (lightest_step > (Search_step / 2)) {
    for (int i = 0; i < Search_step - lightest_step; i++) {
      rotate_angle(-angle_step);
      delay(5);
    }
  } else {
    for (int i = 0; i < lightest_step; i++) {
      rotate_angle(angle_step);
      delay(5);
    }
  }

  stop_motors();
  reset_ticks();

  return true;
}


float read_IR_sensor(float time_interval_ms)
{
  unsigned long start = micros();
  unsigned long duration = (unsigned long)(time_interval_ms * 1e3);  // ms -> µs

  unsigned long count0 = 0;
  unsigned long count1 = 0;

  while (micros() - start < duration) {
      int v = digitalRead(IR_sensor);
      if (v == LOW) count0++;
      else          count1++;
  }

  unsigned long total = count0 + count1;
  if (total == 0) return 0.0;

  return (float)count0 / (float)total;
}

BeaconType detect_beacon()
{
  float ratio_avg = 0.0;
  for (int i = 0; i < 10; i++){
    float ratio = read_IR_sensor(0.1);
    ratio_avg += ratio;    
  }  
  ratio_avg /= 10.0f;

  if (ratio_avg >= 0.27 && ratio_avg <= 0.32)
      return B600;

  if (ratio_avg >= 0.40 && ratio_avg <= 0.45)
      return B1500;

  return None;
}

bool try_search_goal()
{

}
