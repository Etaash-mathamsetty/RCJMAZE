

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include "robot.h"
#include "helpers.h"
#include "debug.h"

#ifndef _DRIVER_H_
#define _DRIVER_H_

#define UNIMP(name, return_val) \
		if(is_simulation) std::cout << "SIM: "; \
		std::cout << "FIXME: unimplemented " << "driver::" << #name << std::endl;	\
		return return_val; 
#define CREATE_DRIVER(return_type, name, ...) static return_type name(__VA_ARGS__)

namespace driver
{
	CREATE_DRIVER(void, init_robot);

	CREATE_DRIVER(void, cleanup);

	CREATE_DRIVER(bool, forward);

	CREATE_DRIVER(void, get_sensor_data);

	CREATE_DRIVER(void, turn_east);

	CREATE_DRIVER(void, turn_west);

	CREATE_DRIVER(void, turn_to, DIR dir);

	CREATE_DRIVER(bool, get_vic);
};

#endif



