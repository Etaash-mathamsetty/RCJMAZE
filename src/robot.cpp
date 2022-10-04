#include "robot.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include <algorithm>
#include "driver.h"
#include "globals.h"

static robot* instance = NULL;

robot::robot(){
#ifdef SIMULATION
		//FIXME: use the same value as non simulation
		node* find = std::find_if(nodes, nodes + horz_size * vert_size, [](node node) -> bool{return node.bot;});
		index = find - nodes;
#else
		index = get_index(19,19);
#endif

		//default direction
		dir = DIR::S;
#ifdef SIMULATION
		map = new node[horz_size * vert_size];
#else
		node node;
		map.Push(node);
		driver::get_sensor_data();
#endif

#ifdef DEBUG
	std::cout << "INFO: Debug info is ENABLED" << std::endl;
#else
	std::cout << "INFO: Debug info is DISABLED" << std::endl;
#endif

	instance = this;
}

robot::~robot()
{
#ifdef SIMULATION
	delete[] map;
#endif
}

robot* robot::get_instance()
{
	if(!instance)
		instance = new robot();
	return instance;
}

bool robot::forward(){
	if(!driver::forward())
		return false;
	else
		driver::get_sensor_data();
	return true;
}
