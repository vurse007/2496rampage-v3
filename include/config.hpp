#pragma once
#include "lynx/lynx.hpp"

namespace global {

    inline pros::Imu imu(15);
    inline pros::Rotation verticalPod(16);
    inline pros::Rotation horizontalPod(19);

    inline lynx::PID heading_correction {
        {0,0,0},
        {0,0,0},
        0, 
        0, 
        0, 
        0, 
        0, 
        0, 
        0
    };
    
    inline lynx::PID turn_pid {
        {0,0,0},
        {0,0,0},
        0, 
        0, 
        0, 
        0, 
        0, 
        0, 
        0
    };
    
    inline lynx::PID drive_pid {
        {0,0,0},
        {0,0,0},
        0, 
        0, 
        0, 
        0, 
        0, 
        0, 
        0
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
        'E', // PTO piston port (pistonA)
        {    // PTO shiftable motors (extraA): even index = left, odd index = right
            {11, pros::v5::MotorGears::blue},
            {20, pros::v5::MotorGears::blue}
        }
    };

    inline pros::Controller con(pros::E_CONTROLLER_MASTER);

    inline pros::adi::Pneumatics wing('A', false, false);
    inline pros::adi::Pneumatics hood('B', false, false);
    inline pros::adi::Pneumatics sunroof('C', false, false);
    inline pros::adi::Pneumatics matchloader('D', false, false);
}
