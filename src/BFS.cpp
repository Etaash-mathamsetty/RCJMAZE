#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "link-list.h"
#include "driver.h"
#include "globals.h"
#include "debug.h"
#include "helpers.h"
#include "simulation.h"

//returns shortest possible path to the nearest unvisited tile
Queue<int> BFS(robot& robot)
{
	int* parent = (int*)alloca(horz_size * vert_size * sizeof(int));
	LinkedList<int> worker;
	Queue<int> path;
	int cur_index = robot.index;
	do
	{
		nearest_quad quad = robot.get_nearest(cur_index);
		//std::sort(quad.nearest, quad.nearest + 4);
		for(int i = 0; i < 4; i++)
		{
			if(quad[i] != -1)
				worker.push_back(quad[i]);
		}
		parent[worker[0].value] = robot.index;

		//continue working on this
		worker.pop_front();
	}while(worker.size() > 0);

	return path;
}

int main(int argc, char* argv[]){
	driver::init_robot();
#ifdef SIMULATION
	if(argc > 1)
		sim::read_map_from_file(argv[1]);
	else
		sim::read_map_from_file("field.txt");
#endif
	robot& robot = *robot::get_instance();
	driver::get_sensor_data();
	printf("node[%d]\n", robot.index);
	debug::print_node(nodes[robot.index]);

	//print_node(nodes[robot.index]);
	debug::print_map();
	std::cout << (std::string)robot.get_nearest() << std::endl;
	//alr BFS time

	//Queue<int> path = BFS(robot);

	//DRIVER_turn_west();
	//std::cout << (std::string)robot << std::endl;
	//print_node(nodes[robot.index]);
	#ifdef SIMULATION
		std::cout << "Autnomous? (y/n):";
		std::string x;
		std::cin >> x;
		if(tolower(x[0]) == 'n')
			while(sim::run_command()){ debug::print_map(); }
		else
		{
			//BFS code goes here
		}
	#else 
		//REAL CODE HERE

	#endif
	//DRIVER_turn_east();
	//std::cout << (std::string)robot << std::endl;

	driver::cleanup();
	return 0;
}
