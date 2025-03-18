#ifndef _TOPICS_H_
#define _TOPICS_H_

#include "rodos.h"
#include "matlib.h"

struct test_matrix
{
  RODOS::Matrix_<3,3> ABC{(double []){},"D"};
  RODOS::Matrix_<3,3> DEF{(double []){1},"H"};
};

struct data_tof_range
{
  int d[4] = {0, 0, 0, 0}; // Distance, mm
  float v[4] = {0, 0, 0, 0}; // Velocity, mm/s
  float dm = 0.0; // Winsorized mean of distances, mm
  float vm = 0.0; // Winsorized mean of velocity, mm/s
  float dt = 0; // Thread period, millis
  bool approach = false; // true if satellites approach each other.
  int status = 0;
};

struct data_current_ctrl
{
  float i[4]; // Current, milli Amp
  float dt = 0; // Thread period, millis
};

struct data_collision_ctrl
{
  float dk[2]; // Distance gains: {kp, ki}
  float vk[2]; // Velocity gains: {kp, ki}
  float dt = 0; // Thread period, millis
};

struct data_desired_current
{
  float i[4]; // milli Amp
  RODOS::Vector_<4> SS{(double []){0},{"U"}};
};

extern Topic<data_tof_range> topic_tof_range;
extern Topic<data_current_ctrl> topic_current_ctrl;
extern Topic<data_collision_ctrl> topic_collision_ctrl;
extern Topic<data_desired_current> topic_desired_current;

extern Topic<data_desired_current> test_data;

#endif // telecommand.h
