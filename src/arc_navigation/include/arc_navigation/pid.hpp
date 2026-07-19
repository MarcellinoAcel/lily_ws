#pragma once
#include <algorithm>

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd),
          integral_(0.0), prev_error_(0.0), initialized_(false) {}

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        initialized_ = false;
    }

    void setGains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    // Returns correction output. Call at fixed dt intervals.
    double compute(double error, double dt) {
        if (!initialized_) {
            prev_error_ = error;
            initialized_ = true;
        }

        integral_ += error * dt;

        // Anti-windup: clamp integral contribution
        const double integral_limit = 0.5;
        integral_ = std::clamp(integral_, -integral_limit, integral_limit);

        double derivative = (dt > 0.0) ? (error - prev_error_) / dt : 0.0;
        prev_error_ = error;

        return (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
    }

private:
    double kp_, ki_, kd_;
    double integral_;
    double prev_error_;
    bool initialized_;
};