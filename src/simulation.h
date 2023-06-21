#ifndef SIMULATION_H_INCLUDED
#define SIMULATION_H_INCLUDED

#include <fstream>
#include <iostream>
#include <vector>
#include "robot.h"
#include "globals.h"
#include "helpers.h"

namespace sim
{

#ifdef SIMULATION
    inline int _horz_size;
    inline int _vert_size;
    inline int sim_robot_index;
    inline std::vector<int> up_ramp_positions[max_num_floors];
    inline std::vector<int> down_ramp_positions[max_num_floors];
    inline bool just_went_on_ramp = false;

    void read_map_from_file(std::string name);

    int get_down_ramp_index(int cur_index);

    int get_up_ramp_index(int cur_index);

    bool run_command();

#endif
};

#endif // SIMULATION_H_INCLUDED
