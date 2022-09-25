#include "robot.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include <algorithm>
#include "driver.h"

robot::robot(){
#ifdef SIMULATION
		//FIXME: use the same value as non simulation
		node* find = std::find_if(nodes, nodes + horz_size * vert_size, [](node node) -> bool{return node.bot;});
		index = find - nodes;
#else
		index = get_index(20,20);
#endif

		//default direction
		dir = DIR::S;
#ifdef SIMULATION
		map = new node[horz_size * vert_size];
#else
		node node;
		map.Push(node);
		DRIVER_get_sensor_data();
#endif
}

bool robot::forward(){
	if(!driver::forward())
		return false;
	else
		driver::get_sensor_data();
	return true;
}
