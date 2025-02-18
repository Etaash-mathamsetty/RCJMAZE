#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <iostream>
#include <sstream>
#include <string.h>
#include <future>
#include <vector>
#include "globals.h"
#include "helpers.h"
#include "scripting.h"

class robot
{
	protected:
		robot();

		~robot(); 
	public:
		DIR dir;
		int index;
		node* map;
		node** floors;
		bool floors_vis[max_num_floors] = {false};
		int start_tile_floor[max_num_floors] = {0};

		//helper funcs
		bool forward();

		nearest_quad get_nearest()
		{
			return helper::get_nearest(this->index);
		}

		explicit operator std::string()
		{
			std::stringstream x;
			x << "Robot Info: ";
			x << "DIR: " << helper::dir_to_string(dir) << ", ";
			x << "Index: " << index;
			return x.str();
		}

		static robot* get_instance();
};

#endif // ROBOT_H_INCLUDED
