

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include "robot.h"
#include "helpers.h"

#ifndef _DRIVER_H_
#define _DRIVER_H_

#define CREATE_DRIVER_UNIMP(return_type, name, return_val) \
	static return_type DRIVER##_##name() \
	{ \
		if(is_simulation) std::cout << "SIM: "; \
		std::cout << "FIXME: unimplemented " << "driver::" << #name << std::endl;	\
		return return_val; \
	}

#define CREATE_DRIVER(return_type, name, ...) static return_type name(__VA_ARGS__)


inline robot* bot;

namespace driver
{
#ifdef SIMULATION
	//create SIMULATION drivers here

	CREATE_DRIVER(void, init_robot)
	{
		bot = new robot();
	}

	CREATE_DRIVER(void, cleanup)
	{
		delete bot;
		delete[] nodes;
	}

	CREATE_DRIVER(bool, forward)
	{
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
#ifdef DEBUG
					HELPER_print_robot_debug_info(bot);
#endif
					return false;
				}
				break;
			}
			case DIR::E:
			{
				if(bot->index < horz_size && !(bot->map[bot->index].E)){
					bot->map[bot->index].bot = false;
					nodes[bot->index].bot = false;
					(bot->index)++;
					bot->map[bot->index].bot = true;
					nodes[bot->index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
#ifdef DEBUG
					HELPER_print_robot_debug_info(bot);
#endif
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
#ifdef DEBUG
					HELPER_print_robot_debug_info(bot);
#endif
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
#ifdef DEBUG
					HELPER_print_robot_debug_info(bot);
#endif
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
		CHECK(bot);
		CHECK(bot->map);
		CHECK(nodes);
		bot->map[bot->index].vis = true;
		memcpy(bot->map + bot->index, nodes + bot->index, sizeof(node));
	}

	CREATE_DRIVER(void, turn_east)
	{
		CHECK(bot);
		if(bot->dir == DIR::W)
			bot->dir = DIR::N;
		else
			//can't do dir++ so...
			bot->dir = DIR(int(bot->dir)+1);
	}

	CREATE_DRIVER(void, turn_west)
	{
		CHECK(bot);
		if(bot->dir == DIR::N)
			bot->dir = DIR::W;
		else
			//can't do dir-- so...
			bot->dir = DIR(int(bot->dir)-1);
	}

	CREATE_DRIVER(void, turn_to, DIR dir)
	{
		CHECK(bot);
		bot->dir = dir;
	}

	CREATE_DRIVER(bool, get_vic){
		CHECK(bot);
		CHECK(bot->map);
		return bot->map[bot->index].vic;
	}

#else
	//create ACTUAL drivers here



#endif

};

#endif



