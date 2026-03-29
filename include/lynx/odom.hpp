#pragma once

#include "drive.hpp"

namespace lynx {

    class odom_drive : public drive {
        public:
            class point {
                public:
                    double x;
                    double y;
                    double theta;
                    bool has_theta;

                    point(double x, double y);
                    point(double x, double y, double theta);
            };

            enum class odom_type{ DUAL, VERT, HORIZ };

            odom_type o_type;

            double theta_deg = 0.0;
            double prev_horizontal_in = 0.0;
            double prev_vertical_in   = 0.0;
            double prev_theta         = 0.0;

            double imu_heading_offset_deg = 0.0;

            double horizontal_encoder_raw = 0.0;
            double vertical_encoder_raw   = 0.0;

            point current_pos{0.0, 0.0, 0.0};

            pros::Task* odom_task = nullptr;

            odom_drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd = 2);

            ~odom_drive();

            double pod_ticks_to_inches(double ticks) const;

            void read_sensors();

            void reset(double nx = 0.0, double ny = 0.0);
            void reset(double nx = 0.0, double ny = 0.0, double nt = 0.0);

            void update();


            //motion algorithms are defined under the src/motion folder
            //~

            void mtp(double x, double y, int timeout, double scale, bool forwards = true, double close_threshold = 7.5);
            void mtp(double x, double y, double theta, double lead=0.6, int timeout, double scale, bool forwards = true, double close_threshold = 7.5);
    };
}
