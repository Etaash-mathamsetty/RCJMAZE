#include <iostream>
#include <sstream>
#include <string.h>
#include "globals.h"



#ifndef _ROBOT
#define _ROBOT

struct nearest_quad
{
	int nearest[4];

	int& operator[] (int x)
	{
		return nearest[x];
	}

	explicit operator std::string()
	{
		std::stringstream x;
		x << "E: " << nearest[0] << ", W: " << nearest[1] << ", S: " << nearest[2] << ", N: " << nearest[3];
		return x.str();
	}
};

class robot{
	public:
		DIR dir;
		int index;
#ifdef SIMULATION
		node* map;
#else
		Queue<node> map;
#endif

		robot();

		~robot(){
#ifdef SIMULATION
			delete[] map;
#endif
		}

		//helper funcs
		bool forward();

		nearest_quad get_nearest(int _index)
		{
			nearest_quad quad;
			//std::cout << ((is_valid_index((int)index-1) && !map[index].W) ? index-1 : 0u) << std::endl;
			quad[0] = ((is_valid_index(_index+1) && !map[_index].E) ? _index+1 : -1);
			quad[1] = ((is_valid_index(_index-1) && !map[_index].W) ? _index-1 : -1);
			quad[2] = ((is_valid_index(_index+horz_size) && !map[_index].S) ? _index+horz_size : -1);
			quad[3] = ((is_valid_index(_index-horz_size) && !map[_index].N) ? _index-horz_size : -1);
			return quad;
		}

		nearest_quad get_nearest()
		{
			return get_nearest(index);
		}


		explicit operator std::string(){
			std::stringstream x;
			x << "Robot Info: ";
			x << "DIR: " << dir_to_string(dir) << ", ";
			x << "Index: " << index;
			return x.str();
		}

};
#endif
