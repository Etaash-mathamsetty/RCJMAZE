#include <iostream>
#include <sstream>
#include <string.h>
#include <future>
#include "globals.h"
#include "helpers.h"
#include "scripting.h"

#ifndef _ROBOT
#define _ROBOT

class robot{
	protected:
		robot();

		~robot(); 
	public:
		DIR dir;
		int index;
		node* map;

		//helper funcs
		bool forward();

		nearest_quad get_nearest()
		{
			return helper::get_nearest(this->index);
		}

		explicit operator std::string(){
			std::stringstream x;
			x << "Robot Info: ";
			x << "DIR: " << helper::dir_to_string(dir) << ", ";
			x << "Index: " << index;
			return x.str();
		}

		static robot* get_instance();
};
#endif
