#pragma once
#include "auton.hpp"
#include "main.h"
#include <vector>

using namespace pros;

int auton_selector(std::vector<Auton>& autons, pros::Controller& controller);