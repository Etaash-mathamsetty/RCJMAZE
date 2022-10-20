#include "driver.h"
#include "robot.h"
#include "scripting.h"
#include <chrono>
#include <thread>

namespace driver
{
    #ifdef SIMULATION

    CREATE_DRIVER(void, init_robot)
	{
		//init robot, and ignore pointer return value
		CHECK(robot::get_instance());
		PythonScript::initPython();
		PythonScript::Exec(init_py_file);
	}

    CREATE_DRIVER(void, cleanup)
	{
		delete[] nodes;
		//delete robot::get_instance();
		PythonScript::Exec(cleanup_py_file);
	}    

    CREATE_DRIVER(bool, forward)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		CHECK(nodes);
		//FIXME: too much repitition
		//simulate time for movement
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		switch(bot->dir){
			case DIR::N:
			{
				if(helper::is_valid_index(bot->index - horz_size) && helper::is_valid_index(sim::sim_robot_index - horz_size) && !bot->map[bot->index].N){
					bot->map[bot->index].bot = false;
					nodes[sim::sim_robot_index].bot = false;
					bot->index -= horz_size;
					sim::sim_robot_index -= horz_size;
					bot->map[bot->index].bot = true;
					nodes[sim::sim_robot_index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
			case DIR::E:
			{
				if(helper::is_valid_index(bot->index + 1) && helper::is_valid_index(sim::sim_robot_index + 1) && !(bot->map[bot->index].E)){
					bot->map[bot->index].bot = false;
					nodes[sim::sim_robot_index].bot = false;
					(bot->index)++;
					sim::sim_robot_index++;
					bot->map[bot->index].bot = true;
					nodes[sim::sim_robot_index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
			case DIR::S:
			{
				if(helper::is_valid_index(bot->index + horz_size) && helper::is_valid_index(sim::sim_robot_index + horz_size) && !bot->map[bot->index].S){
					bot->map[bot->index].bot = false;
					nodes[sim::sim_robot_index].bot = false;
					bot->index += horz_size;
					sim::sim_robot_index += horz_size;
					bot->map[bot->index].bot = true;
					nodes[sim::sim_robot_index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
			case DIR::W:
			{
				if(helper::is_valid_index(bot->index - 1) && helper::is_valid_index(sim::sim_robot_index - 1) && !(bot->map[bot->index].W)){
					bot->map[bot->index].bot = false;
					nodes[sim::sim_robot_index].bot = false;
					(bot->index)--;
					sim::sim_robot_index--;
					bot->map[bot->index].bot = true;
					nodes[sim::sim_robot_index].bot = true;
				}
				else
				{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
		}
		//bot->map[bot->index].vis = true;
		return true;
	}

	CREATE_DRIVER(void, get_sensor_data)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		CHECK(nodes);
		nodes[sim::sim_robot_index].vis = true;
		memcpy(bot->map + bot->index, nodes + sim::sim_robot_index, sizeof(node));
	}

    CREATE_DRIVER(void, turn_east)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		turn_to(helper::next_dir(bot->dir));
	}
    	
	CREATE_DRIVER(void, turn_west)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		turn_to(helper::prev_dir(bot->dir));
	}

    CREATE_DRIVER(void, turn_to, DIR dir)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		bot->dir = dir;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

/*
    CREATE_DRIVER(bool, get_vic){
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		return bot->map[bot->index].vic;
	}
*/

    #else

	CREATE_DRIVER(void, init_robot)
	{
		CHECK(robot::get_instance());
		PythonScript::initPython();
		PythonScript::Exec(init_py_file);
	}

	CREATE_DRIVER(void, cleanup)
	{
		PythonScript::Exec(cleanup_py_file);
	}

	CREATE_DRIVER(void, get_sensor_data)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		bot->map[bot->index].vis = true;
		PythonScript::Exec(cv_py_file);
		PythonScript::Exec(ser_py_file);
		//bot->map[bot->index].letter = (uint8_t)Bridge::get_data_value("letter")[0];
		bot->map[bot->index].N = (bool)Bridge::get_data_value("NWall")[0];
		bot->map[bot->index].E = (bool)Bridge::get_data_value("EWall")[0];
		bot->map[bot->index].S = (bool)Bridge::get_data_value("SWall")[0];
		bot->map[bot->index].W = (bool)Bridge::get_data_value("WWall")[0];

	}

    #endif
} // namespace driver
