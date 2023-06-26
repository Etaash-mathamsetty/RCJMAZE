#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

#include "robot.h"
#include "helpers.h"
#include "globals.h"
#include "simulation.h"
#include <iostream>
#include <list>

namespace debug
{

	void print_node(const node& node);

	void print_robot_info(robot* bot);

	void print_map();

    void print_path(std::list<int>& path);
};

#endif // DEBUG_H_INCLUDED
