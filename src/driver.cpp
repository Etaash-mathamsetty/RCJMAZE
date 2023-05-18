#include "driver.h"
#include "robot.h"
#include "scripting.h"
#include "helpers.h"
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
		in >> num_floors;
		if(num_floors > 1 && second_floor == nullptr)
			second_floor = new simulation_node*[num_second_floors];
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
		if(num_floors > 1)
		{
			for(int l = 0; l < num_floors; l++)
			for(int i = 0; i < horz_size * vert_size; i++)
			{
				in >> n >> s >> e >> w >> vic >> bot_here >> vis >> ramp >> checkpoint;
				bot->second_floor[l][i].N = n;
				bot->second_floor[l][i].S = s;
				bot->second_floor[l][i].E = e;
				bot->second_floor[l][i].W = w;
				bot->second_floor[l][i].bot = bot_here;
				bot->second_floor[l][i].vic = vic;
				bot->second_floor[l][i].vis = vis;
				bot->second_floor[l][i].ramp = ramp;
				bot->second_floor[l][i].checkpoint = checkpoint;

				in >> n >> s >> e >> w >> vic >> bot_here >> vis >> ramp >> checkpoint >> black;
				second_floor[l][i].N = n;
				second_floor[l][i].S = s;
				second_floor[l][i].E = e;
				second_floor[l][i].W = w;
				second_floor[l][i].vic = vic;
				second_floor[l][i].vis = vis;
				second_floor[l][i].ramp = ramp;
				second_floor[l][i].black = black;
				second_floor[l][i].bot = bot_here;
				second_floor[l][i].checkpoint = checkpoint;
			}
		}
	}

	CREATE_DRIVER(void, save_state)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		std::ofstream out;
		out.open("save.txt");
		out << num_floors << std::endl;
		for(int i = 0; i < horz_size * vert_size; i++)
		{
			out << bot->map[i].N << " " << bot->map[i].S << " " << bot->map[i].E << " " << bot->map[i].W
			<< " " << bot->map[i].vic << " " << bot->map[i].bot << " " << bot->map[i].vis << " " << bot->map[i].ramp 
			<< " " << bot->map[i].checkpoint << std::endl;
			out << nodes[i].N << " " << nodes[i].S << " " << nodes[i].E << " " << nodes[i].W
			<< " " << nodes[i].vic << " " << nodes[i].bot << " " << nodes[i].vis << " " << nodes[i].ramp
			<< " " << nodes[i].checkpoint << " " << nodes[i].black << std::endl;
		}
		if(num_floors > 1)
		{
			for(int l = 0; l < num_floors; l++)
			for(int i = 0; i < horz_size * vert_size; i++)
			{
				out << bot->second_floor[l][i].N << " " << bot->second_floor[l][i].S << " ";
				out << bot->second_floor[l][i].E << " " << bot->second_floor[l][i].W << " ";
				out << bot->second_floor[l][i].vic << " " << bot->second_floor[l][i].bot << " ";
				out << bot->second_floor[l][i].vis << " " << bot->second_floor[l][i].ramp << " ";
				out << bot->second_floor[l][i].checkpoint << std::endl;

				out << second_floor[l][i].N << " " << second_floor[l][i].S << " " << second_floor[l][i].E << " ";
				out << second_floor[l][i].W << " " << second_floor[l][i].vic << " " << second_floor[l][i].bot << " ";
				out << second_floor[l][i].vis << " " << second_floor[l][i].ramp << " " << second_floor[l][i].checkpoint << " ";
				out << second_floor[l][i].black << std::endl;
			}
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
		bool ramp = nodes[sim::sim_robot_index].ramp;
		switch(bot->dir)
		{
			case DIR::N:
			{
				if(helper::is_valid_index(bot->index - horz_size) && helper::is_valid_index(sim::sim_robot_index - horz_size) && !bot->map[bot->index].N){
					bot->map[bot->index].bot = false;
					nodes[sim::sim_robot_index].bot = false;
					bot->index -= horz_size;
					sim::sim_robot_index -= horz_size;
					bot->map[bot->index].bot = true;
					nodes[sim::sim_robot_index].bot = true;

					if(ramp)
					{
						floor_num++;
						nodes = second_floor[floor_num - 1];
						sim::sim_robot_index = sim::second_floor_entrance[floor_num - 1];
					}
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
				if(helper::is_valid_index(bot->index + 1) && helper::is_valid_index(sim::sim_robot_index + 1) && !(bot->map[bot->index].E))
				{
					bot->map[bot->index].bot = false;
					nodes[sim::sim_robot_index].bot = false;
					(bot->index)++;
					sim::sim_robot_index++;
					bot->map[bot->index].bot = true;
					nodes[sim::sim_robot_index].bot = true;

					if(ramp) 
					{
						floor_num++;
						nodes = second_floor[floor_num - 1];
						sim::sim_robot_index = sim::second_floor_entrance[floor_num - 1];
					}
				}
				else
				{
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

					if(ramp)
					{
						floor_num++;
						nodes = second_floor[floor_num - 1];
						sim::sim_robot_index = sim::second_floor_entrance[floor_num - 1];
					}
				}
				else
				{
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

					if(ramp)
					{
						floor_num++;
						nodes = second_floor[floor_num - 1];
						sim::sim_robot_index = sim::second_floor_entrance[floor_num - 1];
					}
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

	CREATE_DRIVER(void, drop_vic, int num, bool left)
	{
		//d [drop] N [number of kits, single digit only, can be 0 which will trigger the led] l/r [direction] \n
		std::string direction = left ? "l" : "r";
		PythonScript::CallPythonFunction<bool, std::string>("SendSerialCommand", com::drop_vic + std::to_string(num) + direction + "\n");
	}

	void notify_wall_read()
	{
		//tell the mega pi to send the wall data now
		PythonScript::CallPythonFunction<bool, std::string>("SendSerialCommand", "r\n");
	}

	/* COULD HANG WITH INCORRECT TAG, ONLY USE FOR SERIAL
	only returns the first element of the array, so don't use for everything */
	template<typename T>
	T wait_for_data(const std::string& tag, bool remove = true)
	{
		while(!Bridge::get_data_value(tag).has_value())
		{
			PythonScript::Exec(ser_py_file);
			std::this_thread::sleep_for(std::chrono::milliseconds(80));
		}
		T data = (T)(*Bridge::get_data_value(tag))[0];
		if(remove)
		Bridge::remove_data_value(tag);
		return data;
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

			notify_wall_read();

			bot->map[bot->index].N = wait_for_data<bool>("W", false);
			bot->map[bot->index].E = (bool)(*Bridge::get_data_value("W"))[1];
			bot->map[bot->index].S = (bool)(*Bridge::get_data_value("W"))[2];
			bot->map[bot->index].W = (bool)(*Bridge::get_data_value("W"))[3];
			Bridge::remove_data_value("W");
			bot->map[bot->index].checkpoint = wait_for_data<bool>("CP");
		}
		//debug::print_node(bot->map[bot->index]);

		PythonScript::Exec(cv_py_file);
		bool victim = (bool)(*Bridge::get_data_value("victim"))[0];
		bool left = (bool)(*Bridge::get_data_value("left"))[0];
		int num_rescue = (int)(*Bridge::get_data_value("NRK"))[0];
		if(victim && !bot->map[bot->index].vic)
		{
			drop_vic(num_rescue, left);
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
		PythonScript::CallPythonFunction<bool, std::string>("SendSerialCommand", forward);

		Bridge::remove_data_value("forward_status");

		//wait for it to start running, increase update rate for forward :D
		while(!Bridge::get_data_value("forward_status").has_value()) 
		{ 
			PythonScript::Exec(ser_py_file); 
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		//wait for it to finish running
		while((bool)(*Bridge::get_data_value("forward_status"))[0]) 
		{ 
			PythonScript::Exec(ser_py_file);
			PythonScript::Exec(cv_py_file);
			bool victim = (*Bridge::get_data_value("victim"))[0];
			bool left = (*Bridge::get_data_value("left"))[0];
			int nrk = (*Bridge::get_data_value("NRK"))[0];
			if(victim && !bot->map[bot->index].vic)
			{
				drop_vic(nrk, left);
				bot->map[bot->index].vic = true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		bool status = (bool)(*Bridge::get_data_value("forward_status"))[1];

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

			std::cout << "driver::forward: WARN: Black tile detected, returning false" << std::endl;
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
		turn_cmd += helper::dir_to_char(dir);
		turn_cmd += '\n';
		//std::cout << int(dir) << " " << helper::dir_to_char(dir) << std::endl;
		PythonScript::CallPythonFunction<bool, std::string>("SendSerialCommand", turn_cmd);

		Bridge::remove_data_value("turn_status");
		while(!Bridge::get_data_value("turn_status").has_value())
		{
			PythonScript::Exec(ser_py_file);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		while((bool)(*Bridge::get_data_value("turn_status"))[0])
		{
			PythonScript::Exec(ser_py_file);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	}

    #endif
} // namespace driver
