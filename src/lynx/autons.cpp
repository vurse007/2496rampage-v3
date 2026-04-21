#include "autons.hpp"
#include "config.hpp"
#include "utility.hpp"
#include "queue.hpp"

Auton* auton = nullptr;
std::string names;
lynx::Queue queue;
using namespace global;

void left_half() {
    queue.start();
    chassis.reset(0,0,0);
    chassis.straight(31.75, 1200, 0.7);
    pros::delay(50);
    chassis.turn_abs(-90, 600);
    matchloader.set_value(true);
    pros::delay(400);
    lynx::utility::storage(127);
    chassis.straight(22.5, 900, 0.35);
    pros::delay(350);
    queue.schedule_delay(300, []() {
        lynx::utility::low_goal(0);
    });
    chassis.straight(-31, 900, 0.55);
    lynx::utility::low_goal(127);
    queue.schedule_delay(300, []() {
        chassis.straight(-3, 300);
    });
    pros::delay(50);
    lynx::utility::long_goal(127);
    pros::delay(1350);
    global::matchloader.set_value(false);
    chassis.straight(18.5, 800, 0.65);
    pros::delay(100);
    chassis.turn_abs(136, 800);
    lynx::utility::storage(127);
    pros::delay(200);

    chassis.straight(30, 900, 0.65);
    global::matchloader.set_value(true);
    pros::delay(1000);
    global::matchloader.set_value(false);

    chassis.turn_abs(-43, 900);
    pros::delay(100);
    chassis.straight(-24, 800, 0.70);
    lynx::utility::low_goal(127);
    pros::delay(25);    
    lynx::utility::middle_goal(127);
    pros::delay(1000);
    chassis.turn_abs(-41, 300);
    chassis.straight(32, 1000, 0.65);
    pros::delay(300);
    chassis.turn_abs(-90, 800, 0.95);
    lynx::utility::long_goal(0);

    global::wing.set_value(false);
    queue.schedule_delay(500, []() {
        chassis.set_state(lynx::DriveState::CHASSIS_8);
    });
    chassis.set_brake_mode(MOTOR_BRAKE_HOLD);

    chassis.straight(-24, 2500, 0.55);
}

void right_half(){
    queue.start();
    chassis.reset(0,0,0);
    chassis.straight(31.25, 1200, 0.7);
    pros::delay(50);
    chassis.turn_abs(90, 600);
    global::matchloader.set_value(true);
    pros::delay(200);
    lynx::utility::storage(127);
    chassis.straight(22.5, 900, 0.35);
    pros::delay(400);
    queue.schedule_delay(300, []() {
        lynx::utility::low_goal(0);
    });
    chassis.straight(-31, 900, 0.55);
    lynx::utility::low_goal(-127);
    queue.schedule_delay(300, []() {
        chassis.straight(-3, 300);
    });
    pros::delay(50);
    lynx::utility::long_goal(127);
    chassis.turn_abs(90, 1000);
    pros::delay(1350);
    global::matchloader.set_value(false);
    chassis.straight(15, 800, 0.65);
    pros::delay(100);
    chassis.turn_abs(-136, 800);
    lynx::utility::storage(127);
    pros::delay(200);

    chassis.straight(30, 900, 0.65);
    global::matchloader.set_value(true);
    pros::delay(1000);
    global::matchloader.set_value(false);

    //chassis.turn_abs(-142, 900);
    pros::delay(100);
    chassis.straight(12.5, 800, 0.60);
    // chassis.turn_abs(-149, 300);
    lynx::utility::low_goal(-127);
    pros::delay(1000);
    chassis.turn_abs(-136, 300);
    global::matchloader.set_value(false);

    chassis.straight(-32, 1000, 0.65);
    pros::delay(400);
    chassis.turn_abs(-90, 800, 0.95);
    lynx::utility::low_goal(0);

    global::wing.set_value(false);
    queue.schedule_delay(500, []() {
        chassis.set_state(lynx::DriveState::CHASSIS_8);
    });
    chassis.set_brake_mode(MOTOR_BRAKE_HOLD);

    chassis.straight(25, 2500, 0.55);
}

void min_awp(){
    chassis.straight(4,1000, 0.8);
}

void blank() {}

Auton leftHalf("left_half     ", "Red   ", left_half, "red");
Auton Blank("blank        ", "      ", blank, "");
Auton MinAWP("min_awp      ", "      ", min_awp, "");


std::vector<Auton> autons = {
    leftHalf,
    Blank,
    MinAWP
}; 

