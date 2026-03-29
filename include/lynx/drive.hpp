#pragma once
#include "main.h"
#include "pid.hpp"

namespace lynx {
    struct motor_specs {
        int port;
        pros::v5::MotorGears gearset;

        motor_specs(int p, pros::v5::MotorGears g);
    };

    class group{
        private:
            std::vector<std::shared_ptr<pros::Motor>> motors;
        
        public:
            group(const std::vector<motor_specs>& motor_specs);

            void set_brake_mode(pros::motor_brake_mode_e_t mode);

            void move(int voltage);
            
            void tare();

            std::shared_ptr<pros::Motor> get_motor(int index) const;

            double get_avg_pos() const;

            double get_avg_temp() const;

            const std::vector<std::shared_ptr<pros::Motor>>& get_motors() const;
    };

    void apply_to_group(group& motor_group, const std::function<void(std::shared_ptr<pros::Motor>)>& func);

    class drive{
        public:
            group left;
            group right;

            const double wheel_diameter;
            const double external_gear_ratio;
            const double track_width;

            const double pod_wheel_diameter;

            const double horizontal_offset;
            const double vertical_offset;

            pros::Imu* imu;
            pros::Rotation* vertical_pod;
            pros::Rotation* horizontal_pod;

            PID* heading_corrector;
            PID* turn_pid;
            PID* drive_pid;

            drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd = 2);

            virtual ~drive() = default;

            void set_brake_mode(pros::motor_brake_mode_e mode);

            virtual void move(int left_velocity, int right_velocity);

            void tare();

            double get_position() const;

            double get_left_pos();

            double get_right_pos();

            void apply_to_drive(const std::function<void(std::shared_ptr<pros::Motor>)>& func);

            

            //motion algorithms are defined under the src/motion folder
            //~

            //basic motions:
            std::optional<double> UNIVERSAL_TARGET_HEADING;
            void straight(double target, int timeout, double scale);
            void turn_abs(double target, int timeout, double scale);
            void turn_rel(double delta_deg, int timeout, double scale);
    };

}