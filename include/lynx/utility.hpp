#pragma once

#include "main.h"
#include <cmath>
#include <cstdint>

namespace lynx {

    namespace utility{

        double degrees_to_radians(double deg);
        double radians_to_degrees(double rad);
        double wrap_to_pi(double angle);

        class timer {
            public:
                uint32_t start_time = pros::millis();
                uint32_t target_time;
                bool running = false;

                timer(uint32_t t=0);

                void start();

                uint32_t elapsed() const;

                void reset();

                void restart();

                bool has_elapsed(uint32_t ms=0);

                void stop();
        };

        //get inches helpers
        double rotation_to_inches(double ticks, double wheel_diameter);
        double motor_to_inches(double ticks, double wheel_diameter, pros::v5::MotorGears cartridge = pros::MotorGears::blue);

        //absolute turn logic helper
        double get_abs_rot_err(double target, double current);
    }
    
}