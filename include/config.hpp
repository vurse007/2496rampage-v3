#pragma once
#include "lynx/lynx.hpp"

namespace global {

    inline pros::Imu imu(15);
    inline pros::Rotation verticalPod(-16);
    inline pros::Rotation horizontalPod(19);

    inline lynx::PID heading_correction {
        {6.7, 0.01, 0},      // general_constants: kp, ki, kd
        {0, 0, 0},      // refined_constants
        0,                // refined_range
        0,                  // settle range
        127,               // slew
        30,                 // integral_threshold
        200,                // max_integral
        0,                  // deadband
        10000
    };

    inline lynx::PID turn_pid {
        {6, 0.0001, 48},      // general_constants: kp, ki, kd
        {6, 0.0001, 48},      // refined_constants
        3,                // refined_range
        1,                  // settle range
        127,               // slew
        7,                 // integral_threshold
        200,                // max_integral
        0,                  // deadband
        500                 //settle timer
    };

    inline lynx::PID drive_pid {
        {10, 0.0001, 15},      // general_constants: kp, ki, kd
        {10, 0.0001, 15},      // refined_constants
        3,                  //refined range
        1,                  // settle range
        127,               // slew
        7,                 // integral_threshold
        200,                // max_integral
        0,                  // deadband
        500                 // settle timer
    };

    inline lynx::odom_drive chassis {
        {
            {14, pros::v5::MotorGears::blue},
            {-13, pros::v5::MotorGears::blue},
            {-12, pros::v5::MotorGears::blue}
        },
        {
            {-17, pros::v5::MotorGears::blue},
            {18, pros::v5::MotorGears::blue},
            {19, pros::v5::MotorGears::blue}
        },
        2.75,
        0.8,
        11.5,
        &heading_correction,
        &turn_pid,
        &drive_pid,
        &imu,
        &verticalPod,
        &horizontalPod,
        0,
        0,
        2,
        63, //GEOOOOOOOOOOOOORGE RUSSELLL
        207,
        1.0, // settle_dist_tolerance (inches)
        1.0, // settle_heading_tolerance (degrees)
        'D', // PTO piston port (pistonA)
        {    // PTO shiftable motors (extraA): even index = left, odd index = right
            {11, pros::v5::MotorGears::blue},
            {20, pros::v5::MotorGears::blue}
        }
    };

    inline pros::Controller con(pros::E_CONTROLLER_MASTER);

    inline pros::adi::Pneumatics wing('H', false, false);
    inline pros::adi::Pneumatics hood('E', false, false);
    inline pros::adi::Pneumatics sunroof('B', false, false);
    inline pros::adi::Pneumatics matchloader('A', false, false);
}
