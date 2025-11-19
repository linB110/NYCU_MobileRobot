#pragma once
#include <Arduino.h>
#include <stdint.h>

// ---------- Params ----------
#ifndef Search_step
#define Search_step 20
#endif

// ---------- sensor pins ----------
const int light_sensor = A0;
const int IR_sensor = 12;

const int ambient_light = 850;  // change it for different env
const int detected_puck_val = 800; // threshold of detecting puck

enum BeaconType{
  None = 0,
  B1500, 
  B600
};

extern BeaconType searched_beacon;
extern BeaconType target_beacon;
// initialization
void search_sensor_init();

// searching puck
int read_light_sensor(int sample_times);
bool try_search_puck();

// searching_goal
float read_IR_sensor(float time_interval_ms);
BeaconType detect_beacon();
bool try_search_goal();
