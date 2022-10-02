#include <fstream>
#include <iostream>
#include "robot.h"
#include "globals.h"
#include "helpers.h"

#ifndef _SIMULATION_H_
#define _SIMULATION_H_

namespace sim
{

    void read_map_from_file(std::string name);

    bool run_command();
};

#endif