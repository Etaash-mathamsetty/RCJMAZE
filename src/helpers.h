#include "robot.h"
#include <iostream>

//TODO: rename to debug.h and switch to debug namespace

inline void HELPER_print_robot_debug_info(robot* bot)
{
	CHECK(bot);
	std::cout << "DEBUG INFO: " << std::endl;
	std::cout << (std::string)(*bot) << std::endl;
	print_node(bot->map[bot->index]);
}

namespace helper
{

};

namespace debug
{

};
