#pragma once
#include <algorithm>

struct PID
{
    double kp, ki, kd;
    double i_term = 0;
    double last_err = 0;
    double out_min, out_max;

    PID(double kp_, double ki_, double kd_, double min_, double max_)
        : kp(kp_), ki(ki_), kd(kd_), out_min(min_), out_max(max_) {}

    double update(double err, double dt)
    {
        i_term += err * dt;
        double d = (err - last_err) / dt;
        last_err = err;

        double out = kp * err + ki * i_term + kd * d;
        return std::clamp(out, out_min, out_max);
    }

    void reset()
    {
        i_term = 0;
        last_err = 0;
    }
};