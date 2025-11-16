#pragma once
#include <Arduino.h>
#include <stdint.h>

// ---------- Params ----------
#ifndef Search_step
#define Search_step 20
#endif

// ---------- sensor pins ----------
extern const int light_sensor;
extern const int IR_sensor;

const int ambient_light = 900;  // change it for different env
const int light_threshold = 20;

enum BeaconType{
  None = 0,
  B1500, 
  B600
};

BeaconType searched_beacon = None;
BeaconType target_beacon = B1500; // change it if different beacon is set

// initialization
void search_sensor_init();

// searching puck
int read_light_sensor(int sample_times);
bool try_search_puck();

// searching_goal
float read_IR_sensor(float time_interval);
BeaconType detect_beacon();
bool try_search_goal();