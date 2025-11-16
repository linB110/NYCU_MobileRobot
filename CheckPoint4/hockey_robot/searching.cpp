#include "searching.h"
#include "motor_motion.h"

// ---------- Sensor pins  ----------
const int light_sensor = A0;
const int IR_sensor = 12;

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
  // stop motion and stablize
  stop_motors();
  delay(5);

  const float angle_step = 360 / Search_step;
  for (int i = 0; i < Search_step; i++){
    rotate_angle(angle_step);
    delay(5);


  }
}

float read_IR_sensor(float time_interval)
{
  unsigned long start_time = millis();
    unsigned long duration = (unsigned long)(time_interval * 1000);

    long sum = 0;
    int count = 0;

    while (millis() - start_time < duration) {
        int value = analogRead(IR_PIN);
        sum += value;
        count++;
        delay(5); 
    }

    if (count == 0) return 0.0;

    return (float)sum / count;
}

BeaconType detect_beacon()
{

}

bool try_search_goal()
{

}
