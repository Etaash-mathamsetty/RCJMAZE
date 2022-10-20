#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include <math.h>
#include <chrono>
#include <algorithm>
#include "link-list.h"
#include "driver.h"
#include "globals.h"
#include "debug.h"
#include "helpers.h"
#include "simulation.h"

//returns shortest possible path to the nearest unvisited tile
Stack<int> BFS(robot& robot)
{
	int* parent = (int*)alloca(horz_size * vert_size * sizeof(int));
	//set everything to an obv invalid index
	for(int i = 0; i < horz_size * vert_size; i++) { parent[i] = -1; }
	LinkedList<int> worker;
	Stack<int> path;
	int cur_index = robot.index;
	do
	{
		nearest_quad quad = helper::get_nearest(cur_index);
		//std::sort(quad.nearest, quad.nearest + 4);
		for(int i = 0; i < 4; i++)
		{
			if(quad[i] != -1 && parent[cur_index] != quad[i])
			{
				worker.push_back(quad[i]);
				parent[quad[i]] = cur_index;
			}
		}

		assert(worker.size() > 0);
		cur_index = worker[0].value;

		if(!robot.map[cur_index].vis)
		{
			//BFS is done
			break;
		}

		worker.pop_front();
	} while(worker.size() > 0);

	//backtracking
	path.Push(cur_index);
	do
	{
		path.Push(parent[path[0]]);
	} while(path[0] != robot.index);
	path.Pop();

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
	printf("node[%d]:\n", robot.index);
	debug::print_node(robot.map[robot.index]);
	//print_node(nodes[robot.index]);
	debug::print_map();
	std::cout << (std::string)robot.get_nearest() << std::endl;
	//Queue<int> path = BFS(robot);

	//DRIVER_turn_west();
	//std::cout << (std::string)robot << std::endl;
	//print_node(nodes[robot.index]);
	#ifdef SIMULATION
		std::cout << "Autnomous? (y/n):";
		std::string x;
		std::cin >> x;
		bool quitable = false;
		if(tolower(x[0]) == 'n')
			while(sim::run_command()){ debug::print_map(); }
		else
		{
			//BFS code goes here
			std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
			while(start + std::chrono::seconds(200) >= std::chrono::steady_clock::now())
			{
				if(start + std::chrono::seconds(130) < std::chrono::steady_clock::now())
				{
					robot.map[helper::get_index(default_index, default_index)].vis = false;
					quitable = true;
				}
				Stack<int> path = BFS(robot);
				debug::print_path(path);
				for(size_t l = 0; l < path.Size(); l++)
				{
					if(path[l] == robot.index+1)
					{
						driver::turn_to(DIR::E);
						robot.forward();
					}
					else if(path[l] == robot.index-1)
					{
						driver::turn_to(DIR::W);
						robot.forward();
					}
					else if(path[l] == robot.index + horz_size)
					{
						driver::turn_to(DIR::S);
						robot.forward();
					}
					else if(path[l] == robot.index - horz_size)
					{
						driver::turn_to(DIR::N);
						robot.forward();
					}
					debug::print_map();
				}
				if(quitable && robot.index == helper::get_index(default_index, default_index))
					break;
			}
		}
	#else 
		//REAL CODE HERE

	#endif
	//DRIVER_turn_east();
	//std::cout << (std::string)robot << std::endl;

	driver::cleanup();
	return 0;
}
