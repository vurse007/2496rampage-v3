#include "lynx/odom.hpp"
#include "lynx/utility.hpp"

namespace lynx {

    void odom_drive::mtp(double x, double y, int timeout, double scale, bool forwards, double close_threshold){
        drive_pid->reset();
        turn_pid->reset();

        utility::timer safety_timer(timeout);
        safety_timer.start();

        bool close = false;

        while (true) {
            double cx = current_pos.x;
            double cy = current_pos.y;
            double ct = current_pos.theta;

            double dx = x - cx;
            double dy = y - cy;
            double dist_to_target = std::hypot(dx, dy);
            double angle_to_target = atan2(dx, dy);

            double adjusted_theta = forwards ? ct : ct + M_PI;
            double ang_err_rad = utility::wrap_to_pi(angle_to_target - adjusted_theta);
            double lateral_error = dist_to_target * cos(ang_err_rad);

            if (dist_to_target < close_threshold && !close) {
                close = true;
            }

            double angular_error_deg = utility::radians_to_degrees(ang_err_rad);
            double angular_speed = close ? 0.0 : turn_pid->calculate(angular_error_deg, 0, scale);
            double lateral_speed = drive_pid->calculate(lateral_error, 0, scale);

            if (!close && forwards) lateral_speed = std::fmax(lateral_speed, 0);
            else if (!close && !forwards) lateral_speed = std::fmin(lateral_speed, 0);

            double left_power = lateral_speed + angular_speed;
            double right_power = lateral_speed - angular_speed;
            double max_power = 127.0 * scale;
            double ratio = std::max(std::fabs(left_power), std::fabs(right_power)) / max_power;
            if (ratio > 1) {
                left_power /= ratio;
                right_power /= ratio;
            }

            move(static_cast<int>(left_power), static_cast<int>(right_power));

            if (std::fabs(lateral_error) <= drive_pid->settle_range) {
                if (!drive_pid->settle_timer.running)
                    drive_pid->settle_timer.start();
            } else {
                if (drive_pid->settle_timer.running)
                    drive_pid->settle_timer.reset();
            }

            if (drive_pid->settle_timer.running &&
                drive_pid->settle_timer.has_elapsed(drive_pid->settle_timer_target)) {
                break;
            }

            if (safety_timer.has_elapsed()) break;

            pros::delay(10);
        }

        move(0, 0);
    }

    void odom_drive::mtp(double x, double y, double theta, double lead, int timeout, double scale, bool forwards, double close_threshold){
        drive_pid->reset();
        turn_pid->reset();

        utility::timer safety_timer(timeout);
        safety_timer.start();

        double target_theta = utility::wrap_to_pi(utility::degrees_to_radians(theta));
        if (!forwards) target_theta = utility::wrap_to_pi(target_theta + M_PI);

        bool close = false;
        bool lateral_settled = false;

        while (true) {
            double cx = current_pos.x;
            double cy = current_pos.y;
            double ct = current_pos.theta;

            double dx = x - cx;
            double dy = y - cy;
            double dist_to_target = std::hypot(dx, dy);

            if (dist_to_target < close_threshold && !close) {
                close = true;
            }

            // carrot point: offset behind the target along its heading
            double carrot_x = close ? x : x - sin(target_theta) * lead * dist_to_target;
            double carrot_y = close ? y : y - cos(target_theta) * lead * dist_to_target;

            double dcx = carrot_x - cx;
            double dcy = carrot_y - cy;
            double dist_to_carrot = std::hypot(dcx, dcy);
            double angle_to_carrot = atan2(dcx, dcy);

            // angular error: aim at carrot when far, match target heading when close
            double adjusted_theta = forwards ? ct : ct + M_PI;
            double ang_err_rad = close
                ? utility::wrap_to_pi(target_theta - adjusted_theta)
                : utility::wrap_to_pi(angle_to_carrot - adjusted_theta);

            // lateral error: full distance when far (sign only), projected when close
            double angle_diff = utility::wrap_to_pi(angle_to_carrot - ct);
            double lateral_error;
            if (close) {
                lateral_error = dist_to_carrot * cos(angle_diff);
            } else {
                lateral_error = dist_to_carrot * (cos(angle_diff) >= 0 ? 1.0 : -1.0);
            }

            double angular_error_deg = utility::radians_to_degrees(ang_err_rad);
            double angular_speed = turn_pid->calculate(angular_error_deg, 0, scale);
            double lateral_speed = drive_pid->calculate(lateral_error, 0, scale);

            if (!close && forwards) lateral_speed = std::fmax(lateral_speed, 0);
            else if (!close && !forwards) lateral_speed = std::fmin(lateral_speed, 0);

            double left_power = lateral_speed + angular_speed;
            double right_power = lateral_speed - angular_speed;
            double max_power = 127.0 * scale;
            double ratio = std::max(std::fabs(left_power), std::fabs(right_power)) / max_power;
            if (ratio > 1) {
                left_power /= ratio;
                right_power /= ratio;
            }

            move(static_cast<int>(left_power), static_cast<int>(right_power));

            // lateral settle
            if (std::fabs(lateral_error) <= drive_pid->settle_range) {
                if (!drive_pid->settle_timer.running)
                    drive_pid->settle_timer.start();
            } else {
                if (drive_pid->settle_timer.running)
                    drive_pid->settle_timer.reset();
            }

            if (drive_pid->settle_timer.running &&
                drive_pid->settle_timer.has_elapsed(drive_pid->settle_timer_target)) {
                lateral_settled = true;
            }

            // angular settle
            if (std::fabs(angular_error_deg) <= turn_pid->settle_range) {
                if (!turn_pid->settle_timer.running)
                    turn_pid->settle_timer.start();
            } else {
                if (turn_pid->settle_timer.running)
                    turn_pid->settle_timer.reset();
            }

            bool angular_settled = turn_pid->settle_timer.running &&
                turn_pid->settle_timer.has_elapsed(turn_pid->settle_timer_target);

            if (close && lateral_settled && angular_settled) break;

            if (safety_timer.has_elapsed()) break;

            pros::delay(10);
        }

        move(0, 0);
    }

}