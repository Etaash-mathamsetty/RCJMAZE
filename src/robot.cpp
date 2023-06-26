#include "robot.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include <algorithm>
#include "driver.h"
#include "globals.h"

static robot* instance = NULL;

robot::robot(){
		index = helper::get_index(default_index, default_index);
		//default direction
		dir = DIR::N;
		floors = new node*[max_num_floors];
		memset(floors, 0, sizeof(node*) * max_num_floors);
		for(int i = 0; i < max_num_floors; i++)
		{
			floors[i] = new node[horz_size * vert_size];
			memset(floors[i], 0, horz_size * vert_size * sizeof(node));
		}
		map = this->floors[start_floor];
		memset(floors_vis, 0, sizeof(floors_vis));
		floors_vis[start_floor] = true;
		memset(start_tile_floor, 0, sizeof(start_tile_floor));
		start_tile_floor[start_floor] = helper::get_index(default_index, default_index);

#ifdef DEBUG
	std::cout << "INFO: Debug info is ENABLED" << std::endl;
#else
	std::cout << "INFO: Debug info is DISABLED" << std::endl;
#endif
}

robot::~robot()
{
	for(int i = 0; i < max_num_floors; i++)
	{
		if(floors[i])
			delete[] floors[i];
	}
	delete[] floors;
	//delete[] map;
}

//returns singleton instance
robot* robot::get_instance()
{
	if(!instance)
		instance = new robot();
	return instance;
}

//move forward helper
bool robot::forward(){
	if(!driver::forward())
		return false;
	driver::get_sensor_data();
	return true;
}
