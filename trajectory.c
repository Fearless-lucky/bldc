#include "trajectory.h"
#include "math.h"

void traj_init(Trajectory_t *t)
{
    t->running = 0;
}

void traj_plan(Trajectory_t *t, float target, float start, float max_v, float accel)
{
    float dist = target - start;
    float abs_dist = fabsf(dist);
    float d_accel = max_v * max_v / (2.0f * accel);

    t->start_pos = start;
    t->target_pos = target;
    t->accel = accel;
    t->elapsed = 0;
    t->dir = (dist >= 0) ? 1 : -1;

    if (abs_dist <= 2.0f * d_accel) {
        float peak_v = sqrtf(accel * abs_dist);
        t->cruise_v = peak_v;
        t->accel_time = peak_v / accel;
        t->cruise_time = 0;
        t->decel_time = t->accel_time;
    } else {
        t->cruise_v = max_v;
        t->accel_time = max_v / accel;
        t->cruise_time = (abs_dist - 2.0f * d_accel) / max_v;
        t->decel_time = t->accel_time;
    }
    t->total_time = t->accel_time + t->cruise_time + t->decel_time;
    t->running = 1;
}

float traj_step(Trajectory_t *t)
{
    if (!t->running) return t->target_pos;

    t->elapsed += 0.001f;
    float e = t->elapsed;

    if (e <= t->accel_time) {
        float s = 0.5f * t->accel * e * e;
        return t->start_pos + t->dir * s;
    }

    float t1 = t->accel_time;
    float t2 = t1 + t->cruise_time;

    if (e <= t2) {
        float s_accel = 0.5f * t->accel * t1 * t1;
        float s_cruise = t->cruise_v * (e - t1);
        return t->start_pos + t->dir * (s_accel + s_cruise);
    }

    if (e <= t->total_time) {
        float remain = t->total_time - e;
        return t->target_pos - t->dir * 0.5f * t->accel * remain * remain;
    }

    t->running = 0;
    return t->target_pos;
}

uint8_t traj_done(Trajectory_t *t)
{
    return !t->running;
}
