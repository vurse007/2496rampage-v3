#include "main.h"
#include "lynx/utility.hpp"
#include "config.hpp"

// lv_obj_t* image;
// LV_IMAGE_DECLARE(f1);
// LV_IMAGE_DECLARE(lynx_img);
// LV_IMAGE_DECLARE(rampage);


//driver control methods
// ~

double apply_turn_curve(double stick_value) {
    // Input is already in ≈ [-127, 127] from controller
    double abs_x = std::abs(stick_value);
    if (abs_x < 1e-6) return 0.0;  // avoid div-by-zero or noise

    // Your function, computed on positive side only
    double curve_positive = abs_x + 0.00004 * abs_x * (std::pow(abs_x, 2) - 127.0 * 127.0)
                                       / (1.0 + std::pow(abs_x / 50.0, 6));

    // Mirror sign of original stick
    double output = (stick_value >= 0.0) ? curve_positive : -curve_positive;

    // Final safety clamp (shouldn't be necessary, but good practice)
    return std::clamp(output, -127.0, 127.0);
}

void driverCon() {

    double forward = global::con.get_analog(ANALOG_LEFT_Y);
    double turn_raw = global::con.get_analog(ANALOG_RIGHT_X);

    // Deadband (your original logic)
    if (std::abs(forward) < 5)   forward = 0;
    if (std::abs(turn_raw) < 5)  turn_raw = 0;

    // Apply your non-linear curve **only to the turn axis**
    double turn_curved = /*apply_turn_curve(*/turn_raw/*)*/;

    // Classic arcade mixing
    double left_power  = forward + turn_curved;
    double right_power = forward - turn_curved;

    // Final clamping to motor range
    left_power  = std::clamp(left_power,  -127.0, 127.0);
    right_power = std::clamp(right_power, -127.0, 127.0);

    // Send to chassis
    global::chassis.move(static_cast<int>(left_power),
                            static_cast<int>(right_power));

}

void printTemps(){
	lynx::utility::print_info(
        pros::millis(),
        &global::con,
        std::vector<std::string>{"CRT", "CLT", "RT", "LT"},
        std::vector<double>{global::chassis.right.get_avg_temp(), global::chassis.left.get_avg_temp(), global::chassis.get_extra_temp(0), global::chassis.get_extra_temp(1)}
    );
}

void intakeCon(){
	if (global::con.get_digital(DIGITAL_R1)){
        global::hood.set_value(false);
        global::chassis.set_state(lynx::DriveState::CHASSIS_STANDARD);
        global::chassis.move_subgroup(127, -127);
    }
    else if (global::con.get_digital(DIGITAL_R1) && global::con.get_digital(DIGITAL_R2)){
        global::hood.set_value(false);
        global::chassis.set_state(lynx::DriveState::CHASSIS_STANDARD);
        global::chassis.move_subgroup(-127, 127);
        global::sunroof.set_value(true);
    }
    else if (global::con.get_digital(DIGITAL_R2)){
        global::hood.set_value(false);
        global::chassis.set_state(lynx::DriveState::CHASSIS_STANDARD);
        global::chassis.move_subgroup(-127,-127);
    }
    else if (global::con.get_digital(DIGITAL_L2)){
        global::hood.set_value(true);
        global::chassis.set_state(lynx::DriveState::CHASSIS_STANDARD);
        global::chassis.move_subgroup(127, -127);
    }
    else{
        global::chassis.move_subgroup(0,0);
    }
}

void stateCon(){
	if (global::con.get_digital_new_press(DIGITAL_DOWN)){
        global::chassis.set_state(lynx::DriveState::CHASSIS_8);
    }
    
    if (global::con.get_digital_new_press(DIGITAL_B)) global::matchloader.toggle();
    static bool firstToggle = false;
    if (firstToggle){global::wing.set_value(!global::con.get_digital(DIGITAL_L1));}
    else{
        if(global::con.get_digital(DIGITAL_L1)){
            firstToggle = true;
        }
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	global::con.clear();
	// image = lv_image_create(lv_screen_active());
	// lv_image_set_src(image, &f1);
	// lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true){
		driverCon();
		intakeCon();
		stateCon();
		printTemps();
	}
}