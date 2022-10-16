#include "robot.h"
#include "helpers.h"
#include "globals.h"
#include "simulation.h"
#include "link-list.h"
#include <iostream>

#ifndef __DEBUG_H__
#define __DEBUG_H__

namespace debug
{

	void print_node(const node& node);

	void print_robot_info(robot* bot);

	void print_map();

    void print_path(Stack<int>& path);
};

#endif