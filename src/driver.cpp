#include "driver.h"
#include "robot.h"
#include "scripting.h"
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>

namespace driver
{
    #ifdef SIMULATION

	CREATE_DRIVER(void, load_state)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		std::ifstream in;
		in.open("save.txt");
		bool n, s, e, w, vic, bot_here, vis, ramp, checkpoint, black;
		for(int i = 0; i < horz_size * vert_size; i++)
		{
			in >> n >> s >> e >> w >> vic >> bot_here >> vis >> ramp >> checkpoint;
			bot->map[i].N = n;
			bot->map[i].S = s;
			bot->map[i].E = e;
			bot->map[i].W = w;
			bot->map[i].vic = vic;
			bot->map[i].bot = bot_here;
			bot->map[i].vis = vis;
			bot->map[i].checkpoint = checkpoint;
			if(bot->map[i].bot)
				bot->index = i;
			in >> n >> s >> e >> w >> vic >> bot_here >> vis >> ramp >> checkpoint >> black;
			nodes[i].N = n;
			nodes[i].S = s;
			nodes[i].E = e;
			nodes[i].W = w;
			nodes[i].vic = vic;
			nodes[i].bot = bot_here;
			nodes[i].vis = vis;
			nodes[i].checkpoint = checkpoint;
			nodes[i].black = black;
			if(nodes[i].bot)
				sim::sim_robot_index = i;
		}
	}

	CREATE_DRIVER(void, save_state)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		std::ofstream out;
		out.open("save.txt");
		for(int i = 0; i < horz_size * vert_size; i++)
		{
			out << bot->map[i].N << " " << bot->map[i].S << " " << bot->map[i].E << " " << bot->map[i].W
			<< " " << bot->map[i].vic << " " << bot->map[i].bot << " " << bot->map[i].vis << " " << bot->map[i].ramp 
			<< " " << bot->map[i].checkpoint << std::endl;
			out << nodes[i].N << " " << nodes[i].S << " " << nodes[i].E << " " << nodes[i].W
			<< " " << nodes[i].vic << " " << nodes[i].bot << " " << nodes[i].vis << " " << nodes[i].ramp
			<< " " << nodes[i].checkpoint << " " << nodes[i].black << std::endl;
		}
	}

    CREATE_DRIVER(void, init_robot)
	{
		//init robot, and ignore pointer return value
		CHECK(robot::get_instance());
		PythonScript::initPython();
		PythonScript::Exec(init_py_file);

		if(std::filesystem::exists("save.txt"))
			load_state();
	}

    CREATE_DRIVER(void, cleanup)
	{
		delete[] nodes;
		//delete robot::get_instance();
		PythonScript::Exec(cleanup_py_file);
		if(std::filesystem::exists("save.txt"))
			std::filesystem::remove("save.txt");
	}    

    CREATE_DRIVER(bool, forward)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		CHECK(nodes);
		//FIXME: too much repitition
		//simulate time for movement
		#ifdef SIM_MOV_DELAY
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		#endif
		auto org_index = bot->index;
		auto org_sim_index = sim::sim_robot_index;
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
		if(nodes[sim::sim_robot_index].black)
		{
			bot->map[org_index].bot = true;
			nodes[org_sim_index].bot = true;
			nodes[sim::sim_robot_index].bot = false;
			bot->map[bot->index].bot = false;
			bot->map[bot->index].N = true;
			bot->map[bot->index].E = true;
			bot->map[bot->index].W = true;
			bot->map[bot->index].S = true;
			//W:
			if(helper::is_valid_index(bot->index - 1))
			{
				bot->map[bot->index - 1].E = true;
				// nodes[sim::sim_robot_index - 1].E = true;
			}
			//E:
			if(helper::is_valid_index(bot->index + 1))
			{
				bot->map[bot->index + 1].W = true;
				// nodes[sim::sim_robot_index + 1].W = true;
			}
			//S:
			if(helper::is_valid_index(bot->index + horz_size))
			{
				bot->map[bot->index + horz_size].N = true;
				// nodes[sim::sim_robot_index + horz_size].N = true;
			}
			//N:
			if(helper::is_valid_index(bot->index - horz_size))
			{
				bot->map[bot->index - horz_size].S = true;
				// nodes[sim::sim_robot_index - horz_size].S = true;
			}

			sim::sim_robot_index = org_sim_index;
			bot->index = org_index;

			std::cout << "WARN: Black tile detected, returning false" << std::endl;

			return false;
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
		if(!bot->map[bot->index].vis)
			bot->map[bot->index] = node(nodes[sim::sim_robot_index]);
		if(bot->map[bot->index].checkpoint)
			save_state();
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
		#ifdef SIM_MOV_TIME
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		#endif
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

	CREATE_DRIVER(void, drop_vic, int num)
	{
		//d [drop] N [number of kits, single digit only] \n
		PythonScript::CallPythonFunction<std::string, std::string>("SendSerialCommand", com::drop_vic + std::to_string(num) + "\n");
	}

	CREATE_DRIVER(void, get_sensor_data)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		//bot->map[bot->index].letter = (uint8_t)Bridge::get_data_value("letter")[0];
		if(!bot->map[bot->index].vis)
		{
			bot->map[bot->index].vis = true;
			PythonScript::Exec(cv_py_file);
			PythonScript::Exec(ser_py_file);

			bot->map[bot->index].N = (bool)Bridge::get_data_value("NW")[0];
			bot->map[bot->index].E = (bool)Bridge::get_data_value("EW")[0];
			bot->map[bot->index].S = (bool)Bridge::get_data_value("SW")[0];
			bot->map[bot->index].W = (bool)Bridge::get_data_value("WW")[0];
		}
		int num_rescue = (int)Bridge::get_data_value("NRK")[0];
		if(num_rescue > 0 && !bot->map[bot->index].vic)
		{
			drop_vic(num_rescue);
			bot->map[bot->index].vic = true;
		}
	}

	CREATE_DRIVER(bool, forward)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		auto org_index = bot->index;
		switch(bot->dir)
		{
			case DIR::N:
			{
				if(helper::is_valid_index(bot->index - horz_size) && !bot->map[bot->index].N){
					bot->map[bot->index].bot = false;
					(bot->index) -= horz_size;
					bot->map[bot->index].bot = true;
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
				if(helper::is_valid_index(bot->index + 1) && !bot->map[bot->index].E){
					bot->map[bot->index].bot = false;
					(bot->index)++;
					bot->map[bot->index].bot = true;
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
				if(helper::is_valid_index(bot->index + horz_size) && !bot->map[bot->index].S){
					bot->map[bot->index].bot = false;
					(bot->index) += horz_size;
					bot->map[bot->index].bot = true;
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
				if(helper::is_valid_index(bot->index - 1) && !bot->map[bot->index].W){
					bot->map[bot->index].bot = false;
					(bot->index)--;
					bot->map[bot->index].bot = true;
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
		}
		/*
		 *
		 * aysnc call using serial: PythonScript::CallPythonFunction("SendSerialCommand", "f\n");
		 * robot will check for black tile, and if there is a black tile, the forward call will go back to the previous tile and return false
		 * The robot's index will then be updated to the original index
		 * Print some error/warn message
		 *
		 */
		std::string forward = "";
		forward += com::forward;
		forward += '\n';
		PythonScript::CallPythonFunction<std::string, std::string>("SendSerialCommmand", forward);
		//wait for it to start running
		while(!(bool)Bridge::get_data_value("forward_status")[0]) { PythonScript::Exec(ser_py_file); }
		//wait for it to finish running
		while((bool)Bridge::get_data_value("forward_status")[0]) { PythonScript::Exec(ser_py_file); }

		bool status = (bool)Bridge::get_data_value("forward_status")[1];

		//false for black tile
		if(!status)
		{
			bot->map[org_index].bot = true;
			bot->map[bot->index].bot = false;
			bot->map[bot->index].N = true;
			bot->map[bot->index].E = true;
			bot->map[bot->index].W = true;
			bot->map[bot->index].S = true;
			//W:
			if(helper::is_valid_index(bot->index - 1))
			{
				bot->map[bot->index - 1].E = true;
				// nodes[sim::sim_robot_index - 1].E = true;
			}
			//E:
			if(helper::is_valid_index(bot->index + 1))
			{
				bot->map[bot->index + 1].W = true;
				// nodes[sim::sim_robot_index + 1].W = true;
			}
			//S:
			if(helper::is_valid_index(bot->index + horz_size))
			{
				bot->map[bot->index + horz_size].N = true;
				// nodes[sim::sim_robot_index + horz_size].N = true;
			}
			//N:
			if(helper::is_valid_index(bot->index - horz_size))
			{
				bot->map[bot->index - horz_size].S = true;
				// nodes[sim::sim_robot_index - horz_size].S = true;
			}

			bot->index = org_index;

			std::cout << "WARN: Black tile detected, returning false" << std::endl;
			return false;
		}

		return true;
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
		std::string turn_cmd = "";
		turn_cmd += com::turn;
		turn_cmd += helper::dir_to_char(dir) + '\n';
		PythonScript::CallPythonFunction<std::string, std::string>("SendSerialCommand", turn_cmd);
	}

    #endif
} // namespace driver
