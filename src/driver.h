

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include "robot.h"
#include "helpers.h"
#include "debug.h"

#ifndef _DRIVER_H_
#define _DRIVER_H_

#define CREATE_DRIVER_UNIMP(return_type, name, return_val, ...) \
	static return_type DRIVER##_##name(__VA_ARGS__) \
	{ \
		if(is_simulation) std::cout << "SIM: "; \
		std::cout << "FIXME: unimplemented " << "driver::" << #name << std::endl;	\
		return return_val; \
	}

#define CREATE_DRIVER(return_type, name, ...) static return_type name(__VA_ARGS__)

namespace driver
{
#ifdef SIMULATION
	//create SIMULATION drivers here

	CREATE_DRIVER(void, init_robot)
	{
		//init robot, and ignore pointer return value
		robot::get_instance();
	}

	CREATE_DRIVER(void, cleanup)
	{
		delete[] nodes;
		delete robot::get_instance();
	}

	CREATE_DRIVER(bool, forward)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		CHECK(nodes);
		switch(bot->dir){
			case DIR::N:
			{
				if(bot->index / horz_size > 0 && !bot->map[bot->index].N){
					bot->map[bot->index].bot = false;
					nodes[bot->index].bot = false;
					bot->index -= horz_size;
					bot->map[bot->index].bot = true;
					nodes[bot->index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
			case DIR::E:
			{
				if(bot->index < horz_size * vert_size && !(bot->map[bot->index].E)){
					bot->map[bot->index].bot = false;
					nodes[bot->index].bot = false;
					(bot->index)++;
					bot->map[bot->index].bot = true;
					nodes[bot->index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
			case DIR::S:
			{
				if(bot->index / horz_size < vert_size && !bot->map[bot->index].S){
					bot->map[bot->index].bot = false;
					nodes[bot->index].bot = false;
					bot->index += horz_size;
					bot->map[bot->index].bot = true;
					nodes[bot->index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
			case DIR::W:
			{
				if(bot->index > 0 && !(bot->map[bot->index].W)){
					bot->map[bot->index].bot = false;
					nodes[bot->index].bot = false;
					(bot->index)--;
					bot->map[bot->index].bot = true;
					nodes[bot->index].bot = true;
				}
				else
				{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
		}
		//bot->map[bot->index].vis = true;
		return true;
	}

	CREATE_DRIVER(void, get_sensor_data)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		CHECK(nodes);
		bot->map[bot->index].vis = true;
		memcpy(bot->map + bot->index, nodes + bot->index, sizeof(node));
	}

	CREATE_DRIVER(void, turn_east)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		helper::next_dir(bot->dir);
	}

	CREATE_DRIVER(void, turn_west)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		helper::prev_dir(bot->dir);
	}

	CREATE_DRIVER(void, turn_to, DIR dir)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		bot->dir = dir;
	}

	CREATE_DRIVER(bool, get_vic){
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		return bot->map[bot->index].vic;
	}

#else
	//create ACTUAL drivers here



#endif

};

#endif



