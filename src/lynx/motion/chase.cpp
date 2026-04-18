#include "lynx/odom.hpp"
#include "lynx/utility.hpp"
#include <algorithm>
#include <cmath>

namespace lynx {

    // sin(x)/x with the removable singularity at 0 handled.
    static inline double safe_sinc(double x) {
        if (std::fabs(x) < 1e-6) return 1.0 - x * x / 6.0;
        return std::sin(x) / x;
    }

    // Ramsete follower.
    //
    // Units throughout this function:
    //   positions in inches, headings in radians,
    //   linear velocities in inches/sec, angular velocities in rad/sec.
    // Conversion to motor voltage (0-127) happens only in the final step,
    // using max_speed (inches/sec) as the scaling reference.
    //
    // Pose convention (from odom.cpp):
    //   theta in radians, 0 = +Y (North), CW positive (matches your IMU).
    //   move(left, right) with positive angular command => turn CW (right).
    //
    // Standard (CCW, math-x) Ramsete:
    //   v = v_ref * cos(e_theta) + k * e_x
    //   w = w_ref + k * e_theta + beta * v_ref * sinc(e_theta) * e_y
    // Mirrored to north-0/CW-positive: substitute e_theta_ccw = -e_theta_cw,
    // e_y_ccw = -e_right, w_ccw = -w_cw. All three negations cancel pairwise,
    // cos/sinc are even, so the equations come out identical when expressed
    // with (e_fwd, e_right) and CW-positive angles.
    void odom_drive::chase(const std::vector<Waypoint>& path,
                           double beta, double zeta, int timeout) {
        if (path.size() < 2) return;
        if (max_speed <= 0.0) return;

        utility::timer safety_timer(timeout);
        safety_timer.start();

        const uint32_t start_ms = pros::millis();
        const double path_duration = path.back().time;

        while (true) {
            const double elapsed_s = (pros::millis() - start_ms) / 1000.0;

            const Waypoint ref = BezierPath::sample_at_time(path, elapsed_s);

            const double x  = current_pos.x;
            const double y  = current_pos.y;
            const double th = current_pos.theta;

            const double x_ref  = ref.x;
            const double y_ref  = ref.y;
            const double th_ref = utility::wrap_to_pi(utility::degrees_to_radians(ref.heading));

            // reference speeds: inches/sec and rad/sec
            const double v_ref = ref.velocity;
            // curvature is 1/inches, "positive = left turn" (CCW).
            // In CW-positive convention a left turn is negative angular vel.
            const double w_ref = -v_ref * ref.curvature;

            // pose error in world frame
            const double ex_w = x_ref - x;
            const double ey_w = y_ref - y;
            const double e_theta = utility::wrap_to_pi(th_ref - th);

            // world -> robot frame (forward along heading, right perpendicular)
            const double sin_th = std::sin(th);
            const double cos_th = std::cos(th);
            const double e_fwd   = ex_w * sin_th + ey_w * cos_th;
            const double e_right = ex_w * cos_th - ey_w * sin_th;

            // ramsete gain (units: 1/sec). v_ref in in/s, w_ref in rad/s:
            // beta needs units of (rad^2 / in^2) for dimensional sanity,
            // so treat beta as a unitless tuning knob that just trades off
            // linear vs angular error weighting.
            const double k = 2.0 * zeta * std::sqrt(w_ref * w_ref + beta * v_ref * v_ref);

            // body-frame velocity commands (inches/sec, rad/sec)
            const double v_cmd = v_ref * std::cos(e_theta) + k * e_fwd;
            const double w_cmd = w_ref + k * e_theta
                               + beta * v_ref * safe_sinc(e_theta) * e_right;

            // differential drive decomposition (CW-positive omega):
            // positive w_cmd -> CW rotation -> left wheel faster than right.
            // units: inches/sec
            double v_left_ips  = v_cmd + w_cmd * track_width / 2.0;
            double v_right_ips = v_cmd - w_cmd * track_width / 2.0;

            // convert from inches/sec to motor units (0-127 signed)
            const double ips_to_motor = 127.0 / max_speed;
            double left_motor  = v_left_ips  * ips_to_motor;
            double right_motor = v_right_ips * ips_to_motor;

            // if either wheel saturates, scale both down proportionally so
            // v/w ratio (and hence path shape) is preserved.
            const double mag = std::max(std::fabs(left_motor), std::fabs(right_motor));
            if (mag > 127.0) {
                const double s = 127.0 / mag;
                left_motor  *= s;
                right_motor *= s;
            }

            move(static_cast<int>(left_motor), static_cast<int>(right_motor));

            if (elapsed_s >= path_duration) break;
            if (safety_timer.has_elapsed()) break;

            pros::delay(10);
        }

        move(0, 0);

        // Terminal settle: Ramsete ends on trajectory time, not convergence.
        // If the robot is already inside the shared settle tolerances we skip
        // the handoff entirely (time efficiency). Otherwise we hand off to
        // mtp with parameters derived from the chase() inputs so the settle
        // behavior stays coherent with the caller's intent.
        const Waypoint& goal = path.back();

        const double dx_goal = goal.x - current_pos.x;
        const double dy_goal = goal.y - current_pos.y;
        const double dist_to_goal = std::hypot(dx_goal, dy_goal);

        const double goal_theta_rad = utility::wrap_to_pi(utility::degrees_to_radians(goal.heading));
        const double heading_err_deg = std::fabs(utility::radians_to_degrees(
            utility::wrap_to_pi(goal_theta_rad - current_pos.theta)));

        const bool already_settled =
            dist_to_goal   <= settle_dist_tolerance &&
            heading_err_deg <= settle_heading_tolerance;

        if (!already_settled) {
            // timeout: fixed fraction of caller's budget, bounded so short
            // paths still get a meaningful settle window.
            const int settle_timeout = std::clamp(timeout / 4, 500, 2000);

            // scale: match the approach speed the profile ended at, so a
            // gentle arrival yields a gentle settle. floor keeps authority
            // if the profile terminated near zero velocity.
            const double settle_scale = std::clamp(
                goal.velocity / std::max(1.0, max_speed), 0.4, 1.0);

            // lead: tied to terminal path curvature. straight endings want
            // a small lead (crisp snap); curved endings want a larger lead
            // to preserve the approach heading.
            const double settle_lead = std::clamp(
                0.2 + 20.0 * std::fabs(goal.curvature), 0.2, 0.5);

            mtp(goal.x, goal.y, goal.heading,
                settle_lead, settle_timeout, settle_scale);
        }
    }

    void odom_drive::chase(std::initializer_list<BezierPath::ControlPoint> pts,
                           double beta, double zeta,
                           double tangent_scale, double spacing_inches,
                           int timeout) {
        auto path = make_path(pts, tangent_scale, spacing_inches);
        chase(path, beta, zeta, timeout);
    }

}
