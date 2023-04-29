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
Stack<int> BFS(robot& robot)
{
	//set everything to an obv invalid index
	int parent[horz_size * vert_size];
	for(int i = 0; i < horz_size * vert_size; i++)
		parent[i] = -1;
	LinkedList<int> worker;
	Stack<int> path;
	int cur_index = robot.index;
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
			robot.map[helper::get_index(default_index, default_index)].vis = false;
			return BFS(robot);
		}

		cur_index = worker[0].value;

		// BFS is done
		if(!robot.map[cur_index].vis)
			break;
		
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
	//setup signal handler
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = [](int s){ driver::cleanup(); /* should clean everything up anyway */ exit(1);};
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

#ifdef SIMULATION
	second_floor = new simulation_node*[num_second_floors];


	for(int i = 1; i < num_second_floors + 1; i++)
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
				Stack<int> path = BFS(*robot);
				debug::print_path(path);
				for(size_t l = 0; l < path.Size(); l++)
				{
					if(path[l] == robot->index+1)
					{
						driver::turn_to(DIR::E);
						robot->forward();
					}
					else if(path[l] == robot->index-1)
					{
						driver::turn_to(DIR::W);
						robot->forward();
					}
					else if(path[l] == robot->index + horz_size)
					{
						driver::turn_to(DIR::S);
						robot->forward();
					}
					else if(path[l] == robot->index - horz_size)
					{
						driver::turn_to(DIR::N);
						robot->forward();
					}
					debug::print_map();
				}
				if(quitable && robot->index == helper::get_index(default_index, default_index))
					break;
			}
		}
	#else 
		//#define TEST_MODE
		//REAL CODE HERE
		while(true)
		{
			Stack<int> path = BFS(*robot);
			debug::print_path(path);
			for(size_t l = 0; l < path.Size(); l++)
			{
				switch(path[l] - robot->index)
				{
					case 1:
						driver::turn_to(DIR::E);
						robot->forward();
						break;
					case -1:
						driver::turn_to(DIR::W);
						robot->forward();
						break;
					case -horz_size:
						driver::turn_to(DIR::N);
						robot->forward();
						break;
					case horz_size:
						driver::turn_to(DIR::S);
						robot->forward();
						break;
				}
			}
			debug::print_node(robot->map[robot->index]);
			if(quitable && robot->index == helper::get_index(default_index, default_index))
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
