#pragma once
#include "main.h"
#include "pid.hpp"

#include <optional>
#include <vector>

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

    // -------------------------------------------------------------------------
    // DriveState — logical PTO transmission modes.
    //   CHASSIS_STANDARD : extraA routes to the secondary subsystem (e.g. intake)
    //   CHASSIS_8        : extraA assists the drivetrain
    // Only meaningful when the chassis was constructed with the PTO overload.
    //
    // extraA may contain any number of motors. In CHASSIS_8 the first half of
    // the motor list (by index) is driven as the left side and the remainder
    // as the right side; for odd counts the extra motor goes to the right.
    // In CHASSIS_STANDARD, extraA is driven via move_subgroup(): even indices
    // use the left velocity, odd indices use the right (matches config extraA).
    // -------------------------------------------------------------------------
    enum class DriveState { CHASSIS_STANDARD, CHASSIS_8 };

    class drive{
        public:
            group left;
            group right;

            const double wheel_diameter;
            const double external_gear_ratio;
            const double track_width;

            const double pod_wheel_diameter;
            const double vertical_offset;
            const double horizontal_offset;
            

            pros::Imu* imu;
            pros::Rotation* vertical_pod;
            pros::Rotation* horizontal_pod;

            PID* heading_corrector;
            PID* turn_pid;
            PID* drive_pid;

            // ---- optional PTO (only populated by the PTO-enabled constructor) ----
            std::optional<pros::adi::Pneumatics> pistonA;
            std::optional<group>                 extraA;
            DriveState                           curr_state = DriveState::CHASSIS_STANDARD;
            bool                                 has_pto    = false;

            // standard constructor — no PTO
            drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, PID* hc, PID* tp, PID* dp, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd = 2);

            // PTO-enabled constructor — adds a piston + 2-motor shiftable group
            drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, PID* hc, PID* tp, PID* dp, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd, char pistonA_port, const std::vector<motor_specs>& extraA_specs);

            virtual ~drive() = default;

            void set_brake_mode(pros::motor_brake_mode_e_t mode);

            virtual void move(int left_velocity, int right_velocity);

            void tare();

            double get_position() const;

            double get_left_pos();

            double get_right_pos();

            void apply_to_drive(const std::function<void(std::shared_ptr<pros::Motor>)>& func);

            // ---- PTO API (no-ops when has_pto == false) ----
            void       set_state(DriveState s);
            DriveState get_state() const { return curr_state; }
            /** Per-motor velocities in CHASSIS_STANDARD: even index = left, odd = right. */
            void       move_subgroup(int left_power, int right_power);
            /** Same velocity on every motor (delegates to move_subgroup(power, power)). */
            void       move_subgroup(int power);
            double     get_extra_temp(int index) const;

            //motion algorithms are defined under the src/motion folder
            //~

            //basic motions:
            std::optional<double> UNIVERSAL_TARGET_HEADING;
            void straight(double target, int timeout, double scale);
            void turn_abs(double target, int timeout, double scale);
            void turn_rel(double delta_deg, int timeout, double scale);
    };

}