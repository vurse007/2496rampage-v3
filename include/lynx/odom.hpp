#pragma once

#include "drive.hpp"
#include "bezier.hpp"

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

            // motion profiling limits (used by bezier path generation)
            double max_speed;
            double max_accel;

            // shared settle tolerances for odometry-based motions (mtp, chase).
            // distance in inches, heading in degrees. a motion is considered
            // "settled" when lateral/heading error stay inside these bands.
            double settle_dist_tolerance;
            double settle_heading_tolerance;

            point current_pos{0.0, 0.0, 0.0};

            pros::Task* odom_task = nullptr;

            // standard constructor — no PTO
            odom_drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, PID* hc, PID* tp, PID* dp, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd, const double max_speed, const double max_accel, const double settle_dist_tol = 1.0, const double settle_heading_tol = 1.0);

            // PTO-enabled constructor — adds a piston + 2-motor shiftable group
            odom_drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, PID* hc, PID* tp, PID* dp, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd, const double max_speed, const double max_accel, const double settle_dist_tol, const double settle_heading_tol, char pistonA_port, const std::vector<motor_specs>& extraA_specs);

            ~odom_drive();

        private:
            // shared post-construction setup (o_type detection + odom task spawn)
            void init_odom_runtime();

        public:

            double pod_ticks_to_inches(double ticks) const;

            void read_sensors();

            void reset(double nx = 0.0, double ny = 0.0);
            void reset(double nx = 0.0, double ny = 0.0, double nt = 0.0);

            void update();


            //motion algorithms are defined under the src/motion folder
            //~

            void mtp(double x, double y, int timeout, double scale, bool forwards = true, double close_threshold = 7.5);
            void mtp(double x, double y, double theta, double lead, int timeout, double scale, bool forwards = true, double close_threshold = 7.5);

            // bezier path wrappers: forward the chassis's profile limits + track_width
            std::vector<Waypoint> make_path(std::initializer_list<BezierPath::ControlPoint> pts, double tangent_scale = 0.4, double spacing_inches = 0.5) const {
                return BezierPath::make(pts, tangent_scale, spacing_inches, max_speed, max_accel, track_width);
            }

            std::vector<Waypoint> auto_path(const std::vector<std::pair<double,double>>& xy_points, double velocity = 100.0, double tangent_scale = 0.4, double spacing_inches = 0.5) const {
                return BezierPath::auto_path(xy_points, velocity, tangent_scale, spacing_inches, max_speed, max_accel, track_width);
            }

            std::vector<Waypoint> path_from_control_points(const std::vector<BezierPath::ControlPoint>& pts, double tangent_scale = 0.4, double spacing_inches = 0.5) const {
                return BezierPath::from_control_points(pts, tangent_scale, spacing_inches, max_speed, max_accel, track_width);
            }

            // follow a pre-generated path (user already called make_path / auto_path)
            void chase(const std::vector<Waypoint>& path,
                double beta = 2.0,
                double zeta = 0.7,
                int timeout = 5000);

            // generate + follow in one call
            void chase(std::initializer_list<BezierPath::ControlPoint> pts,
                double beta = 2.0,
                double zeta = 0.7,
                double tangent_scale = 0.4,
                double spacing_inches = 0.5,
                int timeout = 5000);
            
            };
}
