#include "odom.hpp"
#include "utility.hpp"
#include <cmath>

namespace lynx {

    odom_drive::point::point(double x, double y): x(x), y(y), has_theta(false) {}
    odom_drive::point::point(double x, double y, double theta): x(x), y(y), theta(theta), has_theta(true){}

    odom_drive::odom_drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, pros::Imu* imu, pros::Rotation* vertical_pod, pros::Rotation* horizontal_pod, const double v_offset, const double h_offset, const double pwd, const double max_speed, const double max_accel)
        : drive(ls, rs, wd, egr, tw, imu, vertical_pod, horizontal_pod, v_offset, h_offset, pwd),
          max_speed(max_speed),
          max_accel(max_accel) {

          if (this->horizontal_pod == nullptr){
            o_type = odom_type::VERT;
          }
          else if (this->vertical_pod == nullptr){
            o_type = odom_type::HORIZ;
          }
          else {
            o_type = odom_type::DUAL;
          }

          odom_task = new pros::Task(
              [](void* param){
                  odom_drive* odom = static_cast<odom_drive*>(param);
                  while(true){
                      odom->update();
                      pros::delay(10);
                  }
              },
              this,
              "odom_drive_task"
          );
        }

    odom_drive::~odom_drive(){
      if(odom_task){
        odom_task->remove();
        delete odom_task;
        odom_task = nullptr;
      }
    }

    double odom_drive::pod_ticks_to_inches(double ticks) const {
      return (ticks / 36000.0) * (M_PI * pod_wheel_diameter);
    }

    void odom_drive::read_sensors(){
      if (o_type != odom_type::VERT)
          horizontal_encoder_raw = horizontal_pod->get_position();
      
      if (o_type != odom_type::HORIZ)
          vertical_encoder_raw = vertical_pod->get_position();
      else {
          vertical_encoder_raw = get_position();
      }

      theta_deg = imu->get_heading();
    }

    void odom_drive::reset(double nx, double ny){
      if (o_type != odom_type::VERT)  horizontal_pod->reset();
      if (o_type != odom_type::HORIZ) vertical_pod->reset();
      else { tare(); }

      current_pos.x = nx;
      current_pos.y = ny;

      read_sensors();

      const double desired_deg = theta_deg;
      imu_heading_offset_deg = theta_deg - desired_deg;
      current_pos.theta = lynx::utility::degrees_to_radians(desired_deg);

      prev_horizontal_in = (o_type != odom_type::VERT) ? pod_ticks_to_inches(horizontal_encoder_raw) : 0.0;
      prev_vertical_in   = pod_ticks_to_inches(vertical_encoder_raw);
      prev_theta         = current_pos.theta;
    }
    void odom_drive::reset(double nx, double ny, double nt){
      if (o_type != odom_type::VERT)  horizontal_pod->reset();
      if (o_type != odom_type::HORIZ) vertical_pod->reset();
      else { tare(); }

      current_pos.x = nx;
      current_pos.y = ny;

      read_sensors();

      const double desired_deg = lynx::utility::radians_to_degrees(nt);
      imu_heading_offset_deg = theta_deg - desired_deg;
      current_pos.theta = nt;

      prev_horizontal_in = (o_type != odom_type::VERT) ? pod_ticks_to_inches(horizontal_encoder_raw) : 0.0;
      prev_vertical_in   = pod_ticks_to_inches(vertical_encoder_raw);
      prev_theta         = current_pos.theta;
    }

    void odom_drive::update(){
      read_sensors();

      // --- heading ---
      double theta_now = lynx::utility::degrees_to_radians(theta_deg - imu_heading_offset_deg);
      if (std::isnan(theta_now) || std::isinf(theta_now)) theta_now = prev_theta;
      theta_now = lynx::utility::wrap_to_pi(theta_now);
      current_pos.theta = theta_now;

      double deltaHeading = lynx::utility::wrap_to_pi(current_pos.theta - prev_theta);
      double avgHeading   = prev_theta + deltaHeading / 2.0;

      // --- raw encoder deltas ---
      const double h_in_now = (o_type != odom_type::VERT) ? pod_ticks_to_inches(horizontal_encoder_raw) : 0.0;
      const double v_in_now = pod_ticks_to_inches(vertical_encoder_raw);

      double dS = (o_type != odom_type::VERT) ? (h_in_now - prev_horizontal_in) : 0.0;
      double dF = v_in_now - prev_vertical_in;

      // --- arc-corrected local displacement ---
      double localX, localY;
      if (std::fabs(deltaHeading) < 1e-6) {
          localX = (o_type != odom_type::VERT) ? (dS - horizontal_offset * deltaHeading) : 0.0;
          localY = dF - vertical_offset * deltaHeading;
      } else {
          double halfSin = 2.0 * std::sin(deltaHeading / 2.0);
          localX = (o_type != odom_type::VERT) ? halfSin * (dS / deltaHeading - horizontal_offset) : 0.0;
          localY = halfSin * (dF / deltaHeading - vertical_offset);
      }

      // --- rotate to global frame ---
      double cosH = std::cos(avgHeading);
      double sinH = std::sin(avgHeading);

      current_pos.x += localX * cosH + localY * sinH;
      current_pos.y += localY * cosH - localX * sinH;

      // --- bookkeeping ---
      if (o_type != odom_type::VERT) prev_horizontal_in = h_in_now;
      prev_vertical_in = v_in_now;
      prev_theta       = current_pos.theta;
    }

}
