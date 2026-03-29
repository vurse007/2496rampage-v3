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

    drive::drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, double pwd):
        left(ls), right(rs), wheel_diameter(wd), external_gear_ratio(egr), track_width(tw), imu(imu), vertical_pod(vertical_pod), horizontal_pod(horizontal_pod), vertical_offset(v_offset), horizontal_offset(h_offset), pod_wheel_diameter(pwd){}

    // helper functions to do common tasks to motors
    void drive::set_brake_mode(pros::motor_brake_mode_e mode) {
        left.set_brake_mode(mode);
        right.set_brake_mode(mode);
    }

    void drive::move(int left_velocity, int right_velocity) {
        left.move(left_velocity);
        right.move(right_velocity);
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

}