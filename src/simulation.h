#ifndef SIMULATION_H_INCLUDED
#define SIMULATION_H_INCLUDED

#include <fstream>
#include <iostream>
#include "robot.h"
#include "globals.h"
#include "helpers.h"

namespace sim
{

    inline int _horz_size;
    inline int _vert_size;
    inline int sim_robot_index;
    inline int second_floor_entrance[num_second_floors] = {-1};

    void read_map_from_file(std::string name);

    bool run_command();
};

#endif // SIMULATION_H_INCLUDED
