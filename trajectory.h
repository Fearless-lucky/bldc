#ifndef __TRAJECTORY_H_
#define __TRAJECTORY_H_

#include "stm32f10x.h"

typedef struct {
    float start_pos;
    float target_pos;
    float accel;
    float cruise_v;
    float accel_time;
    float cruise_time;
    float decel_time;
    float total_time;
    float elapsed;
    int8_t dir;
    uint8_t running;
} Trajectory_t;

void traj_init(Trajectory_t *t);
void traj_plan(Trajectory_t *t, float target, float start, float max_v, float accel);
float traj_step(Trajectory_t *t);
uint8_t traj_done(Trajectory_t *t);

#endif
