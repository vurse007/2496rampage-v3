#pragma once

#include "utility.hpp"

namespace lynx {
    
    struct constants{
        double kp;
        double ki;
        double kd;

        constants(double kp, double ki, double kd);
    };

    class PID{
        public:
            constants general_constants;
            constants refined_constants;

            double refined_range;
            double settle_range;

            int dt = 5;

            double curr = 0.0;
            double tgt = 0.0;

            double error       = 0.0;
            double prev_error  = 0.0;
            double total_error = 0.0;
            double derivative  = 0.0;

            double slew        = 0.0;
            double prev_speed  = 0.0;
            double speed       = 0.0;

            inline static double glb_tgt_heading = 0; // if you use this elsewhere

            double integral_threshold = 0.0;
            double max_integral       = 0.0;
            double deadband           = 0.0;
            double settle_timer_target = 0.0;

            lynx::utility::timer settle_timer;

            PID(constants g, constants r, double rr, double sr, double slew, double it, double mi, double db, double st);

            double calculate(double target, double current, double scale = 1.0);

            void reset();
    };

}