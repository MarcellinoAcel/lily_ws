
#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <algorithm>
#include <cmath>
#include <iostream>

class Kinematic
{
public:
    enum base
    {
        DIFFERENTIAL_DRIVE,
        SKID_STEER,
        MECANUM,
        OMNI,
        OMNI_3
    };

    base base_platform_;

    struct rps
    {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    struct velocities
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct position
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct pwm
    {
        int motor1;
        int motor2;
        int motor3;
        int motor4;
    };
    Kinematic(base robot_base, int motor_max_rps, float max_rps_ratio,
              float motor_operating_voltage, float motor_power_max_voltage,
              float wheel_diameter, float robot_diameter)
        : base_platform_(robot_base),
          wheel_circumference_(M_PI * wheel_diameter),
          robot_circumference_(M_PI * robot_diameter),
          total_wheels_(getTotalWheels(robot_base))
    {

        motor_power_max_voltage = std::clamp<float>(motor_power_max_voltage, 0.0, motor_operating_voltage); 
        max_rps_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rps) * max_rps_ratio;
    }
    velocities getVelocities(float rps1, float rps2, float rps3, float rps4)
    {

        Kinematic::velocities vel;
        float average_rps_x;
        float average_rps_y;
        float average_rps_a;

        if (base_platform_ == DIFFERENTIAL_DRIVE)
        {
            // convert average revolutions per minute to revolutions per second
            rps3 = 0.0;
            rps4 = 0.0;

            average_rps_x = ((float)(rps1 + rps2 + rps3 + rps4) / total_wheels_); // rps
            vel.linear_x = average_rps_x * wheel_circumference_;                  // m/s

            // convert average revolutions per minute to revolutions per second
            average_rps_a = ((float)(-rps1 + rps2 - rps3 + rps4) / total_wheels_);
            vel.angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s
        }
        else if (base_platform_ == OMNI)
        {
            // convert average revolutions per minute to revolutions per second
            average_rps_x = ((float)(-sin(toRad(45)) * rps1 - sin(toRad(135)) * rps2 - sin(toRad(225)) * rps3 - sin(toRad(315)) * rps4) / 2); // rps
            vel.linear_x = average_rps_x * wheel_circumference_;                                                                              // m/s

            // convert average revolutions per minute in y axis to revolutions per second
            average_rps_y = ((float)(cos(toRad(45)) * rps1 + cos(toRad(135)) * rps2 + cos(toRad(225)) * rps3 + cos(toRad(315)) * rps4) / 2);
            vel.linear_y = average_rps_y * wheel_circumference_; // m/s

            // convert average revolutions per minute to revolutions per second
            average_rps_a = ((float)(rps1 + rps2 + rps3 + rps4) / 2);
            vel.angular_z = (average_rps_a * robot_circumference_) / (robot_circumference_); //  rad/s
        }
        else if (base_platform_ == MECANUM)
        {

            // convert average revolutions per minute to revolutions per second
            average_rps_x = ((float)(rps1 + rps2 + rps3 + rps4) / total_wheels_); // rps
            vel.linear_x = average_rps_x * wheel_circumference_;                  // m/s

            // convert average revolutions per minute in y axis to revolutions per second
            average_rps_y = ((float)(-rps1 + rps2 + rps3 - rps4) / total_wheels_); // rps
            vel.linear_y = average_rps_y * wheel_circumference_;                   // m/s

            // convert average revolutions per minute to revolutions per second
            average_rps_a = ((float)(-rps1 + rps2 - rps3 + rps4) / total_wheels_);
            vel.angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s
        }
        else if (base_platform_ == OMNI_3)
        {
            // convert average revolutions per minute to revolutions per second
            average_rps_x = ((float)(-sin(toRad(30)) * rps1 - sin(toRad(150)) * rps2 - sin(toRad(270)) * rps3) / total_wheels_); // rps
            vel.linear_x = average_rps_x * wheel_circumference_;                                                                 // m/s

            // convert average revolutions per minute in y axis to revolutions per second
            average_rps_y = ((float)(cos(toRad(30)) * rps1 + cos(toRad(150)) * rps2 + cos(toRad(270)) * rps3) / total_wheels_);
            vel.linear_y = average_rps_y * wheel_circumference_; // m/s

            // convert average revolutions per minute to revolutions per second
            average_rps_a = ((float)(rps1 + rps2 + rps3 + rps4) / total_wheels_);
            vel.angular_z = (average_rps_a * robot_circumference_) / (robot_circumference_); //  rad/s
        }

        return vel;
    }
    rps getRPS(float linear_x, float linear_y, float angular_z, float imu_angular_z)
    {
        return calculateRPS(linear_x, linear_y, angular_z, imu_angular_z);
    }

    rps getRPS(float linear_x, float linear_y, float angular_z)
    {
        return calculateRPS(linear_x, linear_y, angular_z);
    }
    float getMaxRPS()
    {
        return max_rps_;
    }
    float toRad(float deg)
    {
        return deg * M_PI / 180;
    }
    float toDeg(float rad)
    {
        return rad * 180 / M_PI;
    }

private:
    rps calculateRPS(float linear_x, float linear_y, float angular_z, float imu_angular_z)
    {

        float tangential_vel = angular_z * (robot_circumference_);
        // float tangential_vel = angular_z;

        // convert m/s to m/min
        float linear_vel_x_mins = linear_x;
        float linear_vel_y_mins = linear_y;
        // convert rad/s to rad/min
        float tangential_vel_mins = tangential_vel;

        float x_mps = linear_vel_x_mins / wheel_circumference_;
        float y_mps = linear_vel_y_mins / wheel_circumference_;
        float tan_mps = tangential_vel_mins / wheel_circumference_;

        float a_x_mps = fabs(x_mps);
        float a_y_mps = fabs(y_mps);
        float a_tan_mps = fabs(tan_mps);

        float xy_sum = a_x_mps + a_y_mps;
        float xtan_sum = a_x_mps + a_tan_mps;
        if (xy_sum >= max_rps_ && angular_z == 0)
        {
            float vel_scaler = max_rps_ / xy_sum;

            x_mps *= vel_scaler;
            y_mps *= vel_scaler;
        }

        else if (xtan_sum >= max_rps_ && linear_y == 0)
        {
            float vel_scaler = max_rps_ / xtan_sum;

            x_mps *= vel_scaler;
            tan_mps *= vel_scaler;
        }

        Kinematic::rps rps;
        float rps_motor1, rps_motor2, rps_motor3, rps_motor4;
        // calculate for the target motor rps and direction
        if (base_platform_ == OMNI)
        {
            // front-left motor
            rps_motor1 = -sin(toRad(45 + imu_angular_z)) * x_mps + cos(toRad(45 + imu_angular_z)) * y_mps + robot_radius_ * tan_mps;
            rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

            // rear-left motor
            rps_motor2 = -sin(toRad(135 + imu_angular_z)) * x_mps + cos(toRad(135 + imu_angular_z)) * y_mps + robot_radius_ * tan_mps;
            rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

            // rear-right motor
            rps_motor3 = -sin(toRad(225 + imu_angular_z)) * x_mps + cos(toRad(225 + imu_angular_z)) * y_mps + robot_radius_ * tan_mps;
            rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));

            // front-right motor
            rps_motor4 = -sin(toRad(315 + imu_angular_z)) * x_mps + cos(toRad(315 + imu_angular_z)) * y_mps + robot_radius_ * tan_mps;
            rps.motor4 = fmax(-max_rps_, fmin(rps_motor4, max_rps_));
        }
        else if (base_platform_ == OMNI_3)
        {
            rps_motor1 = -sin(toRad(30)) * x_mps + cos(toRad(30)) * y_mps + robot_radius_ * tan_mps;
            rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

            rps_motor2 = -sin(toRad(150)) * x_mps + cos(toRad(150)) * y_mps + robot_radius_ * tan_mps;
            rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

            rps_motor3 = -sin(toRad(270)) * x_mps + cos(toRad(270)) * y_mps + robot_radius_ * tan_mps;
            rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));

            rps.motor4 = 0;
        }
        else
        {
            // front-left motor
            rps_motor1 = x_mps - y_mps - tan_mps;
            rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

            // front-right motor
            rps_motor2 = x_mps + y_mps + tan_mps;
            rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

            // rear-left motor
            rps_motor3 = x_mps + y_mps - tan_mps;
            rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));

            // rear-right motor
            rps_motor4 = x_mps - y_mps + tan_mps;
            rps.motor4 = fmax(-max_rps_, fmin(rps_motor4, max_rps_));
        }

        return rps;
    }
    rps calculateRPS(float linear_x, float linear_y, float angular_z)
    {

        float tangential_vel = angular_z * (robot_circumference_);
        // float tangential_vel = angular_z;

        // convert m/s to m/min
        float linear_vel_x_mins = linear_x;
        float linear_vel_y_mins = linear_y;
        // convert rad/s to rad/min
        float tangential_vel_mins = tangential_vel;

        float x_mps = linear_vel_x_mins / wheel_circumference_;
        float y_mps = linear_vel_y_mins / wheel_circumference_;
        float tan_mps = tangential_vel_mins / wheel_circumference_;

        float a_x_mps = fabs(x_mps);
        float a_y_mps = fabs(y_mps);
        float a_tan_mps = fabs(tan_mps);

        float xy_sum = a_x_mps + a_y_mps;
        float xtan_sum = a_x_mps + a_tan_mps;
        if (xy_sum >= max_rps_ && angular_z == 0)
        {
            float vel_scaler = max_rps_ / xy_sum;

            x_mps *= vel_scaler;
            y_mps *= vel_scaler;
        }

        else if (xtan_sum >= max_rps_ && linear_y == 0)
        {
            float vel_scaler = max_rps_ / xtan_sum;

            x_mps *= vel_scaler;
            tan_mps *= vel_scaler;
        }

        Kinematic::rps rps;
        float rps_motor1, rps_motor2, rps_motor3, rps_motor4;
        // calculate for the target motor rps and direction
        if (base_platform_ == OMNI)
        {
            // front-left motor
            rps_motor1 = -sin(toRad(45)) * x_mps + cos(toRad(45)) * y_mps + robot_radius_ * tan_mps;
            rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

            // rear-left motor
            rps_motor2 = -sin(toRad(135)) * x_mps + cos(toRad(135)) * y_mps + robot_radius_ * tan_mps;
            rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

            // rear-right motor
            rps_motor3 = -sin(toRad(225)) * x_mps + cos(toRad(225)) * y_mps + robot_radius_ * tan_mps;
            rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));

            // front-right motor
            rps_motor4 = -sin(toRad(315)) * x_mps + cos(toRad(315)) * y_mps + robot_radius_ * tan_mps;
            rps.motor4 = fmax(-max_rps_, fmin(rps_motor4, max_rps_));
        }
        else if (base_platform_ == OMNI_3)
        {
            rps_motor1 = cos(toRad(30)) * x_mps + sin(toRad(30)) * y_mps + robot_radius_ * tan_mps;
            rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

            rps_motor2 = cos(toRad(150)) * x_mps + sin(toRad(150)) * y_mps + robot_radius_ * tan_mps;
            rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

            rps_motor3 = cos(toRad(270)) * x_mps + sin(toRad(270)) * y_mps + robot_radius_ * tan_mps;
            rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));
        }
        else
        {
            // front-left motor
            rps_motor1 = x_mps - y_mps - tan_mps;
            rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

            // front-right motor
            rps_motor2 = x_mps + y_mps + tan_mps;
            rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

            // rear-left motor
            rps_motor3 = x_mps + y_mps - tan_mps;
            rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));

            // rear-right motor
            rps_motor4 = x_mps - y_mps + tan_mps;
            rps.motor4 = fmax(-max_rps_, fmin(rps_motor4, max_rps_));
        }

        return rps;
    }
    int getTotalWheels(base robot_base)
    {
        switch (robot_base)
        {
        case DIFFERENTIAL_DRIVE:
            return 2;
        case SKID_STEER:
            return 41;
        case MECANUM:
            return 43;
        case OMNI:
            return 40;
        case OMNI_3:
            return 30;
        default:
            return 4;
        }
    }
    // keliling roda encoder 4.635 cm
    // 1024 ppr
    float max_rps_;
    float wheels_y_distance_;
    float pwm_res_;
    float wheel_circumference_;
    float robot_circumference_;
    int total_wheels_;
    float robot_radius_ = 0.25;
    float enc_wheel_diameter = 4.635 / 100;
    float total_enc_pulse = 1024;
    float enc_robot_circumference = 20.325 / 100;
    float enc_wheel_circumference = enc_wheel_diameter * M_PI;
};

#endif