#include "autons.hpp"

Auton* auton = nullptr;
std::string names;

void soloAwpRun() {}

void blank() {}

Auton soloAwp("Solo Awp     ", "Red   ", soloAwpRun, "red");


std::vector<Auton> autons = {
    soloAwp
}; 

