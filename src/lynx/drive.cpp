#include "drive.hpp"

namespace lynx {
    motor_specs::motor_specs(int p, pros::v5::MotorGears g): port(p), gearset(g) {}

    group::group(const std::vector<motor_specs>& motor_specs){
        for (const auto& spec : motor_specs){
            auto motor = std::make_shared<pros::Motor>(spec.port, spec.gearset);
            motors.push_back(motor);
        }
    }

    void group::set_brake_mode(pros::motor_brake_mode_e_t mode) {
        for (auto& motor : motors) {
            motor->set_brake_mode(mode);
        }
    }

    void group::move(int voltage) {
        for (auto& motor : motors) {
            motor->move(voltage);
        }
    }

    void group::tare(){
        for (auto& motor : motors) {
            motor->tare_position();
        }
    }

    std::shared_ptr<pros::Motor> group::get_motor(int index) const {
        if (index >=0 && index < motors.size()) {
            return motors[index];
        } else{
            return nullptr;
        }
    }

    double group::get_avg_pos() const {
        double total_pos = 0.0;
        int count = 0;
        for (auto& motor : motors) {
            if (motor) {
                total_pos += motor->get_position();
                count++;
            }
        }
        if (count == 0) return 0.0;
        return total_pos/count;
    }

    double group::get_avg_temp() const {
        double total_temp = 0.0;
        int count = 0;
        for (auto& motor : motors) {
            if (motor) {
                total_temp += motor->get_temperature();
                count++;
            }
        }
        if (count == 0) return 0.0;
        return total_temp/count;
    }

    const std::vector<std::shared_ptr<pros::Motor>>& group::get_motors() const {
        return motors;
    }

    void apply_to_group(group& motor_group, const std::function<void(std::shared_ptr<pros::Motor>)>& func) {
        for (const auto& motor : motor_group.get_motors()) {
            if (motor) func(motor);
        }
    }

    drive::drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, PID* hc, PID* tp, PID* dp, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, double pwd):
        left(ls), right(rs), wheel_diameter(wd), external_gear_ratio(egr), track_width(tw), heading_corrector(hc), turn_pid(tp), drive_pid(dp), imu(imu), vertical_pod(vertical_pod), horizontal_pod(horizontal_pod), vertical_offset(v_offset), horizontal_offset(h_offset), pod_wheel_diameter(pwd){}

    // PTO-enabled constructor: delegate to the standard ctor, then init the PTO bits.
    drive::drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, PID* hc, PID* tp, PID* dp, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd, char pistonA_port, const std::vector<motor_specs>& extraA_specs):
        drive(ls, rs, wd, egr, tw, hc, tp, dp, imu, vertical_pod, horizontal_pod, v_offset, h_offset, pwd)
    {
        pistonA.emplace(pistonA_port, false);
        extraA.emplace(extraA_specs);
        curr_state = DriveState::CHASSIS_STANDARD;
        has_pto    = true;
        pistonA->set_value(false);
    }

    // helper functions to do common tasks to motors
    void drive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
        left.set_brake_mode(mode);
        right.set_brake_mode(mode);
        if (has_pto) extraA->set_brake_mode(mode);
    }

    void drive::move(int left_velocity, int right_velocity) {
        left.move(left_velocity);
        right.move(right_velocity);

        // PTO routing: when the shifter is in CHASSIS_8, the extra motors
        // assist the drivetrain. The first half of extraA is treated as the
        // left side and the remainder as the right side, so the group can be
        // any size (2, 4, 6, ...). For odd counts the extra motor goes right.
        // In CHASSIS_STANDARD the motors belong to the auxiliary subsystem
        // and are driven via move_subgroup instead.
        if (has_pto && curr_state == DriveState::CHASSIS_8) {
            const auto& motors = extraA->get_motors();
            const std::size_t n    = motors.size();
            const std::size_t half = n / 2;
            for (std::size_t i = 0; i < half; ++i)
                if (motors[i]) motors[i]->move(left_velocity);
            for (std::size_t i = half; i < n; ++i)
                if (motors[i]) motors[i]->move(right_velocity);
        }
    }

    void drive::tare() {
        left.tare();
        right.tare();
    }

    double drive::get_position() const{
        if (vertical_pod == nullptr){
            return (left.get_avg_pos() + right.get_avg_pos()) / 2.0;
        } else {
            return vertical_pod->get_position();
        }
    }

    double drive::get_left_pos(){
        return left.get_avg_pos();
    }

    double drive::get_right_pos(){
        return right.get_avg_pos();
    }

    void drive::apply_to_drive(const std::function<void(std::shared_ptr<pros::Motor>)>& func) {
        for (const auto& motor : left.get_motors()) {
            if (motor) func(motor);
        }
        for (const auto& motor : right.get_motors()) {
            if (motor) func(motor);
        }
    }

    // ---- PTO API ----
    void drive::set_state(DriveState s) {
        if (!has_pto) return;
        curr_state = s;
        // CHASSIS_8 → piston extended (extraA into drivetrain),
        // CHASSIS_STANDARD → piston retracted (extraA into auxiliary subsystem)
        pistonA->set_value(s == DriveState::CHASSIS_8);
    }

    void drive::move_subgroup(int power) {
        if (!has_pto) return;
        // extraA only acts as the auxiliary subsystem in CHASSIS_STANDARD.
        if (curr_state != DriveState::CHASSIS_STANDARD) return;
        // All motors run together when used as a subgroup, regardless of size.
        extraA->move(-power);
    }

    double drive::get_extra_temp(int index) const {
        if (!has_pto) return 0.0;
        auto m = extraA->get_motor(index);
        return m ? m->get_temperature() : 0.0;
    }

}