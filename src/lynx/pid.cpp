#include "pid.hpp"

namespace lynx{

    constants::constants(double kp, double ki, double kd): kp(kp), ki(ki), kd(kd) {}

    PID::PID(constants g, constants r, double rr, double sr, double slew_val, double it, double mi, double db, double st): 
        general_constants(g),
        refined_constants(r),
        refined_range(rr),
        settle_range(sr),
        slew(slew_val),
        integral_threshold(it),
        max_integral(mi),
        deadband(db),
        settle_timer_target(st) {}
    
    double PID::calculate(double target, double current, double scale){
        tgt  = target;
        curr = current;

        // --------------------------------
        // ERROR TERMS
        // --------------------------------
        error = tgt - curr;

        if (fabs(error) <= settle_range){
            settle_timer.start();
        }

        double raw_error = error; // save before deadband clamps it

        // Deadband: treat very small error as zero
        if (std::fabs(error) < deadband) {
            error = 0.0;
        }

        // Integral (with threshold & clamping)
        if (std::fabs(error) < integral_threshold) {
            total_error += (error + prev_error) / 2.0;
            total_error = std::clamp(total_error, -max_integral, max_integral);
        }
        // no else — freeze integral when outside threshold

        // Derivative uses raw error so it tapers smoothly into deadband
        derivative = raw_error - prev_error;

        // --------------------------------
        // PICK CONSTANT SET
        // --------------------------------
        const constants& c = (std::fabs(error) < refined_range)
            ? refined_constants
            : general_constants;

        // --------------------------------
        // RAW PID OUTPUT
        // --------------------------------
        double raw_speed = scale * (c.kp * error +
                                    c.ki * total_error +
                                    c.kd * derivative);

        // --------------------------------
        // SLEW LIMITING
        // --------------------------------
        double delta_speed = raw_speed - prev_speed;
        if (delta_speed > slew) {
            speed = prev_speed + slew;
        }
        else if (delta_speed < -slew) {
            speed = prev_speed - slew;
        }
        else {
            speed = raw_speed;
        }

        // Clamp final speed
        speed = std::clamp(speed, -127.0 * scale, 127.0 * scale);

        // Update history
        prev_error = raw_error; // track raw error for next derivative calculation
        prev_speed = speed;

        return speed;
    }

    void PID::reset(){
        error       = 0.0;
        prev_error  = 0.0;
        total_error = 0.0;
        derivative  = 0.0;
        prev_speed  = 0.0;
        speed       = 0.0;
        settle_timer.reset();
    }
}