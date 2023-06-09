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
    inline std::vector<int> ramp_positions[max_num_floors];
    //std::string = "1,0","2,1","3,2","4,3" so on
    //ramp_pair stores the pair 
    inline std::map<std::string, std::vector<ramp_pair>> ramp_pairs;

    void read_map_from_file(std::string name);

    ramp_pair* get_ramp_pair(int floor_num, int index);

    bool run_command();

#endif
};

#endif // SIMULATION_H_INCLUDED
