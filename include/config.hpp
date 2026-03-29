#pragma once
#include "lynx/lynx.hpp"

namespace global {

    inline pros::Imu imu(21);
    inline pros::Rotation verticalPod(20);
    inline pros::Rotation horizontalPod(19);

    inline lynx::odom_drive chassis {
        {
            {1, pros::v5::MotorGears::blue},
            {2, pros::v5::MotorGears::blue},
            {3, pros::v5::MotorGears::blue}
        },
        {
            {4, pros::v5::MotorGears::blue},
            {5, pros::v5::MotorGears::blue},
            {6, pros::v5::MotorGears::blue}
        },
        2.75,
        1,
        12.0,
        &imu,
        &verticalPod,
        &horizontalPod,
        0,
        0
    };

}
