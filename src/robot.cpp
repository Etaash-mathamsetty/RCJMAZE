#include "robot.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include <algorithm>
#include "driver.h"
#include "globals.h"

static robot* instance = NULL;

robot::robot(){
		//simply becomes too complicated if we pretended we were at default_index, default_index in the simulation, am open to ideas tho
		//nvm, im just really freaking dumb, I know how to do it
		index = helper::get_index(default_index,default_index);
		//default direction
		dir = DIR::N;
		map = new node[horz_size * vert_size];
		memset(map, 0, horz_size * vert_size * sizeof(node));

#ifdef DEBUG
	std::cout << "INFO: Debug info is ENABLED" << std::endl;
#else
	std::cout << "INFO: Debug info is DISABLED" << std::endl;
#endif
}

robot::~robot()
{
	delete[] map;
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
