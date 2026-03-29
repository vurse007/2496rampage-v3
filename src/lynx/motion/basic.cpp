#include "lynx/drive.hpp"
#include "lynx/utility.hpp"

namespace lynx{

    void drive::straight(double target, int timeout, double scale){
        drive_pid->settle_timer.reset();
        
        double curr_pos;
        if (!UNIVERSAL_TARGET_HEADING.has_value()){
            UNIVERSAL_TARGET_HEADING = imu->get_heading();
        }
        double target_heading = UNIVERSAL_TARGET_HEADING.value();
        double current_heading = imu->get_heading();
        double init_pos;
        if (vertical_pod != nullptr) {
            init_pos = utility::rotation_to_inches(vertical_pod->get_position(), pod_wheel_diameter);
        }
        else {
            init_pos = utility::motor_to_inches(get_position(), wheel_diameter, pros::v5::MotorGears::blue);
        }

        heading_corrector->reset();
        drive_pid->reset();
        utility::timer safety_timer(timeout);
        safety_timer.start();

        while (true){
            //update current position
            if (vertical_pod != nullptr) {
                curr_pos = utility::rotation_to_inches(vertical_pod->get_position(), pod_wheel_diameter) - init_pos;
            }
            else {
                curr_pos = utility::motor_to_inches(get_position(), wheel_diameter, pros::v5::MotorGears::blue) - init_pos;
            }
            current_heading = imu->get_heading();

            //pid formula
            double drive_speed = drive_pid->calculate(target, curr_pos, scale);

            //heading correction
            double heading_error = utility::get_abs_rot_err(target_heading, current_heading);
            double heading_correction = heading_corrector->calculate(heading_error, 0, scale);

            int left_motor  = static_cast<int>(drive_speed + heading_correction);
            int right_motor = static_cast<int>(drive_speed - heading_correction);
            move(left_motor, right_motor);

            //settling logic
            if (std::fabs(target - curr_pos) <= drive_pid->settle_range) {
                if (!drive_pid->settle_timer.running)
                    drive_pid->settle_timer.start();
            } else {
                if (drive_pid->settle_timer.running)
                    drive_pid->settle_timer.reset();
            }

            if (drive_pid->settle_timer.running &&
                drive_pid->settle_timer.has_elapsed(drive_pid->settle_timer_target))
            {
                break;
            }
            if (safety_timer.has_elapsed()) break;

            pros::delay(10);
        }
        move(0,0);
    }

    void drive::turn_abs(double target, int timeout, double scale){
        double current_heading;

        turn_pid->settle_timer.reset();
        turn_pid->reset();

        utility::timer safety_timer(timeout);
        safety_timer.start();
        
        while(true){
            current_heading = imu->get_heading();
            //get heading and calculate absolute error
            double heading_error = utility::get_abs_rot_err(target, current_heading);

            //pid formula
            double turn_speed = turn_pid->calculate(heading_error, 0, scale);

            //motor outputs
            int left_motor  = static_cast<int>( turn_speed);
            int right_motor = static_cast<int>(-turn_speed);
            move(left_motor, right_motor);

            //settling logic
            if (std::fabs(heading_error) <= turn_pid->settle_range) {
                if (!turn_pid->settle_timer.running)
                    turn_pid->settle_timer.start();
            }
            else {
                // reset settle timer if escaped settle range
                if (turn_pid->settle_timer.running)
                    turn_pid->settle_timer.reset();
            }

            // finish if settle timer exceeded target ms
            if (turn_pid->settle_timer.running &&
                turn_pid->settle_timer.has_elapsed(turn_pid->settle_timer_target))
            {
                break;
            }

            // Safety timeout  
            if (timeout > 0 && safety_timer.has_elapsed())
                break;

            pros::delay(10);
        }
        UNIVERSAL_TARGET_HEADING = target;
        move(0,0);
    }

    void drive::turn_rel(double delta_deg, int timeout, double scale){
        double current = imu->get_heading();
        double target = std::fmod(current + delta_deg + 360.0, 360.0);
        turn_abs(target, timeout, scale);
        //no need to update universal target heading variable -> turn_abs takes care of it
    }

}