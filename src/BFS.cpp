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

void read_map_from_file(){
	std::ifstream in("field.txt");
	if(in.bad())
	{
		std::cerr << "cannot open field.txt" << std::endl;
		return;
	}
	robot& robot = *bot;
	//std::cout << "sizeof DIR: " << sizeof(DIR) << std::endl;
	char x = 0;
	in >> horz_size;
	horz_size++;
	in >> vert_size;
	nodes = new node[horz_size * vert_size];
	//int horz_size;
	//int v = 0;
	in.get(x);
	for(int v = 0; v < vert_size; v++){
		if(v > 0){
			for(int i = 0; i < horz_size; i++){
				//probably wont be necessary on an actual robot, but we need it to read from the field.txt properly
				nodes[helper::get_index(v,i)].N = nodes[helper::get_index(v-1,i)].S;
			}
		}
		//printf("loop 1\n");
		//get rid of extra plus
		//in.get(x);
		if(v == 0)
		for(float i = 0; i < horz_size; i+=0.5){
			in.get(x);
			//printf("x: %c i: %d\n", x, i);
			if(x == '\0')
				return;
			if(x == '\n')
				break;
			if(x == '+') continue;
			node& node = nodes[helper::get_index(v,i)];
			node.N = (x == '-');
			node.bot |= (tolower(x) == 'x');
			node.vis |= node.bot;
			node.vic |= (tolower(x) == 'v');
			if(node.bot)
				robot.index = helper::get_index(v,i);
			//print_node(nodes[get_index(v,i)]);
			//nodes[get_index(v,i)] = node;
			//horz_size++;
		}
		//printf("loop 2\n");
		for(float i = 0; i < horz_size; i+=0.5){
			in.get(x);
			//printf("x: %c\n", x);
			if(x == '\0')
				return;
			if(x == '\n')
				break;
			//if(x == ' ') { i--; continue; }
			node& node = nodes[helper::get_index(v,i)];
			if(x == '|') {
				if((int)i > 0)
				{
					node.W = true;
					nodes[helper::get_index(v,i-1)].E = true;
				}
				else{
					node.W = true;
				}
			}
			node.bot |= (tolower(x) == 'x');
			node.vis |= node.bot;
			node.vic |= (tolower(x) == 'v');
			if(node.bot)
				robot.index = helper::get_index(v,i);
		}
		//printf("loop 3\n");
		for(float i = 0; i < horz_size; i+=0.5){
			in.get(x);
			//printf("x: %c\n", x);
			if(x == '\0')
				return;
			if(x == '\n')
				break;
			if(x == '+') continue;
			node& node = nodes[helper::get_index(v,i)];
			node.S = (x == '-');
			node.bot |= (tolower(x) == 'x');
			node.vis |= node.bot;
			node.vic |= (tolower(x) == 'v');
			if(node.bot)
				robot.index = helper::get_index(v,i);
		}
		//v++;
	}
	return;
}

//returns shortest possible path to the nearest unvisited tile
Queue<int> BFS(robot& robot)
{
	int* parent = (int*)alloca(horz_size * vert_size * sizeof(int));
	Queue<int> worker;
	Queue<int> path;
	int cur_index = robot.index;
	do
	{
		nearest_quad quad = robot.get_nearest(cur_index);
		std::sort(quad.nearest, quad.nearest + 4);
		for(int i = 0; i < 4; i++)
		{
			if(quad[i] != -1)
				worker.Push(quad[i]);
		}
		parent[worker[0]] = robot.index;

		//continue working on this
		worker.PopFront();
	}while(worker.Size() > 0);

	return path;
}

int main(){
	driver::init_robot();
#ifdef SIMULATION
	read_map_from_file();
#endif
	robot& robot = *bot;
	driver::get_sensor_data();
	printf("node[%d]\n", robot.index);
	debug::print_node(nodes[robot.index]);
	//WHY EAST ALL OF A SUDDEN????
	//print_node(nodes[robot.index]);
	debug::print_map();
	std::cout << (std::string)robot.get_nearest() << std::endl;
	//alr BFS time

	//Queue<int> path = BFS(robot);

	//DRIVER_turn_west();
	//std::cout << (std::string)robot << std::endl;
	//print_node(nodes[robot.index]);
	driver::turn_to(DIR::E);
	while(robot.forward())
	{
		debug::print_map();
		//std::cout << (std::string)robot.get_nearest() << std::endl;

	}
	driver::turn_to(DIR::S);
	while(robot.forward())
	{
		debug::print_map();
	}
	//DRIVER_turn_east();
	//std::cout << (std::string)robot << std::endl;

	driver::cleanup();
	return 0;
}
