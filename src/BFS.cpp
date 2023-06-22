#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include <math.h>
#include <chrono>
#include <algorithm>
#include <signal.h>
#include <unistd.h>
#include "link-list.h"
#include "driver.h"
#include "globals.h"
#include "debug.h"
#include "helpers.h"
#include "simulation.h"

bool quitable = false;

//returns shortest possible path to the nearest unvisited tile
Stack<int> BFS()
{
	robot* bot = robot::get_instance();
	//set everything to an obv invalid index
	int parent[horz_size * vert_size];
	for(int i = 0; i < horz_size * vert_size; i++)
		parent[i] = -1;
	LinkedList<int> worker;
	Stack<int> path;
	int cur_index = bot->index;
	do
	{
		if(worker.size() > 0)
			worker.pop_front();
		
		nearest_quad quad = helper::get_nearest(cur_index);
		for(int i = 0; i < 4; i++)
		{
			if(quad[i] != -1 && parent[cur_index] != quad[i])
			{
				worker.push_back(quad[i]);
				parent[quad[i]] = cur_index;
			}
		}

		if(worker.size() == 0)
		{
			/* return to start */
			std::cout << "finished with the maze!" << std::endl;
			std::cout << "returning to starting..." << std::endl;
			quitable = true;
			bot->map[bot->start_tile_floor[floor_num]].vis = false;
			return BFS();
		}

		cur_index = worker[0].value;

		// BFS is done
		if(!bot->map[cur_index].vis)
			break;
		
	} while(worker.size() > 0);


	//backtracking
	path.Push(cur_index);
	do
	{
		path.Push(parent[path[0]]);
	} while(path[0] != bot->index);
	path.Pop();

	return path;
}

int main(int argc, char* argv[]){
	//setup signal handler
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = [](int s){ driver::cleanup(); /* exit() should clean everything else anyway */ exit(1);};
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

#ifdef SIMULATION
	floors = new simulation_node*[max_num_floors];
	memset(floors, 0, sizeof(simulation_node*) * max_num_floors);

	for(int i = 1; i < max_num_floors + 1; i++)
	{
		if(argc > i)
			sim::read_map_from_file(argv[i]);
		else
		{
			if(i > 1)
				sim::read_map_from_file("field" + std::to_string(i) + ".txt");
			else
				sim::read_map_from_file("field.txt");
		}
	}
	nodes = floors[0];
#endif

	driver::init_robot();
	robot* robot = robot::get_instance();
	driver::get_sensor_data();
	printf("node[%d]:\n", robot->index);
	debug::print_node(robot->map[robot->index]);
	//print_node(nodes[robot.index]);
	debug::print_map();
	std::cout << (std::string)robot->get_nearest() << std::endl;
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
			while(true)
			{
				Stack<int> path = BFS();
				debug::print_path(path);
				int old_floor = floor_num;
				for(size_t l = 0; l < path.Size(); l++)
				{
					switch(path[l] - robot->index)
					{
						case 1:
							driver::turn_to(DIR::E);
							break;
						case -1:
							driver::turn_to(DIR::W);
							break;
						case -horz_size:
							driver::turn_to(DIR::N);
							break;
						case horz_size:
							driver::turn_to(DIR::S);
							break;
					}
					robot->forward();
					debug::print_map();
					//entered different floor
					if(old_floor != floor_num)
						break;
				}
				if(quitable && robot->index == helper::get_index(default_index, default_index) && floor_num == start_floor)
					break;
			}
		}
	#else 
		//#define TEST_MODE
		//REAL CODE HERE
		while(true)
		{
			Stack<int> path = BFS();
			debug::print_path(path);
			int old_floor = floor_num;
			for(size_t l = 0; l < path.Size(); l++)
			{
				switch(path[l] - robot->index)
				{
					case 1:
						driver::turn_to(DIR::E);
						break;
					case -1:
						driver::turn_to(DIR::W);
						break;
					case -horz_size:
						driver::turn_to(DIR::N);
						break;
					case horz_size:
						driver::turn_to(DIR::S);
						break;
				}
				robot->forward();
				debug::print_map();
				if(old_floor != floor_num)
					break;
			}
			//debug::print_node(robot->map[robot->index]);
			if(quitable && robot->index == helper::get_index(default_index, default_index) && floor_num == start_floor)
				break;
			#ifdef TEST_MODE
				std::cout << "press any key to continue..." << std::endl;
				getchar();
				printf("\n");
			#endif
		}
	#endif
	//DRIVER_turn_east();
	//std::cout << (std::string)robot << std::endl;

	driver::cleanup();
	return 0;
}
