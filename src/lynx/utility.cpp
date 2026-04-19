#include "utility.hpp"

namespace lynx {
    
    namespace utility{

        double degrees_to_radians(double deg){
            return deg * M_PI / 180.0;
        }
        double radians_to_degrees(double rad){
            return rad * 180.0 / M_PI;
        }
        
        double wrap_to_pi(double angle){
            angle = fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            return angle - M_PI;
        }

        timer::timer(uint32_t t): target_time(t) {}

        void timer::start(){
            start_time = pros::millis();
            running = true;
        }

        uint32_t timer::elapsed() const{
            if (!running) return 0;
            return pros::millis() - start_time;
        }

        void timer::reset(){
            start_time = 0;
            running = false;
        }

        void timer::restart(){
            start();
        }

        bool timer::has_elapsed(uint32_t ms){
            if (ms==0) ms = target_time;
            return elapsed() >= ms;
        }

        void timer::stop(){
            running = false;
        }

        double rotation_to_inches(double ticks, double wheel_diameter){
            return (ticks / 36000.0) * (M_PI * wheel_diameter);
        }

        double motor_to_inches(double ticks, double wheel_diameter, pros::v5::MotorGears cartridge){
            double tps = 300.0;
            if (cartridge == pros::v5::MotorGears::green){
                tps = 900.0;
            }
            else if (cartridge == pros::v5::MotorGears::red){
                tps = 1800.0;
            }
            return (ticks / tps) * (M_PI * wheel_diameter);
        }


        double get_abs_rot_err(double target, double current){
            return fmod((target - current + 540), 360) - 180;
        }

        void print_info(int time, pros::Controller *controller, const std::vector<std::string>& labels, const std::vector<double>& values) {
            if (!controller || labels.empty() || labels.size() != values.size()) return;

            const int pairs_per_line = std::ceil(labels.size() / 3.0);

            auto build_line = [&](int start, int end) -> std::string {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2); // two decimal places
                for (int i = start; i < end && i < (int)labels.size(); ++i) {
                    oss << labels[i] << ": " << values[i];
                    if (i < end - 1 && i < (int)labels.size() - 1) oss << " | ";
                }
                return oss.str();
            };

            if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0) {
                std::string line0 = build_line(0, pairs_per_line);
                controller->print(0, 0, "%s   ", line0.c_str());
            }

            if (time % 100 == 0 && time % 150 != 0) {
                std::string line1 = build_line(pairs_per_line, pairs_per_line * 2);
                controller->print(1, 0, "%s   ", line1.c_str());
            }

            if (time % 150 == 0 && time % 300 != 0) {
                std::string line2 = build_line(pairs_per_line * 2, pairs_per_line * 3);
                controller->print(2, 0, "%s   ", line2.c_str());
            }
        }

    }
    
}