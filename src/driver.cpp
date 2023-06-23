#include "driver.h"
#include "robot.h"
#include "scripting.h"
#include "helpers.h"
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>

void set_ramp_wall()
{
	robot* bot = robot::get_instance();
	CHECK(bot);
	CHECK(bot->map);

	switch(bot->dir)
	{
		case DIR::N:
			bot->map[bot->index].N = true;
			bot->map[bot->index].E = true;
			bot->map[bot->index].W = true;
			return;
		case DIR::E:
			bot->map[bot->index].E = true;
			bot->map[bot->index].N = true;
			bot->map[bot->index].S = true;
			return;
		case DIR::W:
			bot->map[bot->index].W = true;
			bot->map[bot->index].N = true;
			bot->map[bot->index].S = true;
			return;
		case DIR::S:
			bot->map[bot->index].S = true;
			bot->map[bot->index].E = true;
			bot->map[bot->index].W = true;
			return;
	}
}

namespace driver
{
    #ifdef SIMULATION

	CREATE_DRIVER(void, load_state)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		std::ifstream in;
		in.open("save.txt");
		in >> num_floors;
		if(num_floors > 1 && floors == nullptr)
			floors = new simulation_node*[max_num_floors];
		bool n, s, e, w, vic, bot_here, vis, ramp, checkpoint, black;
		for(int l = 0; l < num_floors; l++)
		for(int i = 0; i < horz_size * vert_size; i++)
		{
			int bot_l = l + start_floor;
			in >> n >> s >> e >> w >> vic >> bot_here >> vis >> ramp >> checkpoint;
			bot->floors[bot_l][i].N = n;
			bot->floors[bot_l][i].S = s;
			bot->floors[bot_l][i].E = e;
			bot->floors[bot_l][i].W = w;
			bot->floors[bot_l][i].vic = vic;
			bot->floors[bot_l][i].bot = bot_here;
			bot->floors[bot_l][i].vis = vis;
			bot->floors[bot_l][i].checkpoint = checkpoint;
			if(bot->floors[bot_l][i].bot)
				bot->index = i;
			in >> n >> s >> e >> w >> vic >> bot_here >> vis >> ramp >> checkpoint >> black;
			floors[l][i].N = n;
			floors[l][i].S = s;
			floors[l][i].E = e;
			floors[l][i].W = w;
			floors[l][i].vic = vic;
			floors[l][i].bot = bot_here;
			floors[l][i].vis = vis;
			floors[l][i].checkpoint = checkpoint;
			floors[l][i].black = black;
			if(floors[l][i].bot)
				sim::sim_robot_index = i;
		}
	}

	CREATE_DRIVER(void, save_state)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		std::ofstream out;
		out.open("save.txt");
		out << num_floors << std::endl;
		for(int l = 0; l < num_floors; l++)
		for(int i = 0; i < horz_size * vert_size; i++)
		{
			int bot_l = l + start_floor;
			out << bot->floors[bot_l][i].N << " " << bot->floors[bot_l][i].S << " ";
			out << bot->floors[bot_l][i].E << " " << bot->floors[bot_l][i].W << " ";
			out << bot->floors[bot_l][i].vic << " " << bot->floors[bot_l][i].bot << " ";
			out << bot->floors[bot_l][i].vis << " " << bot->floors[bot_l][i].ramp << " ";
			out << bot->floors[bot_l][i].checkpoint << std::endl;

			out << floors[l][i].N << " " << floors[l][i].S << " " << floors[l][i].E << " ";
			out << floors[l][i].W << " " << floors[l][i].vic << " " << floors[l][i].bot << " ";
			out << floors[l][i].vis << " " << floors[l][i].ramp << " " << floors[l][i].checkpoint << " ";
			out << floors[l][i].black << std::endl;
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

	bool set_mov_indexes(int delta, int ramp_len = 1)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		CHECK(nodes);
		
		bot->map[bot->index].bot = false;
		nodes[sim::sim_robot_index].bot = false;
		bot->index += delta;
		sim::sim_robot_index += delta;
		bot->map[bot->index].bot = true;
		nodes[sim::sim_robot_index].bot = true;

		bool up_ramp = nodes[sim::sim_robot_index].ramp & 0b01;
		bool down_ramp = nodes[sim::sim_robot_index].ramp & 0b10;
		CHECK(up_ramp && down_ramp);
		if(up_ramp)
		{
			set_ramp_wall();
			int down_ramp_index = sim::get_down_ramp_index(sim::sim_robot_index);
			if(down_ramp_index == -1)
			{
				std::cerr << "ERR: no upramp found!" << std::endl;
				return false;
			}
			bot->map[bot->index].bot = false;
			nodes[sim::sim_robot_index].bot = false;
			bot->map[bot->index].ramp = 0b01;
			floor_num++;
			bot->map = bot->floors[floor_num];
			nodes = floors[floor_num - start_floor];
			bot->index += ramp_len * delta;
			sim::sim_robot_index = down_ramp_index;
			bot->map[bot->index].bot = true;
			nodes[sim::sim_robot_index].bot = true;
			bot->map[bot->index - delta].ramp = 0b10;
			
			if(!bot->floors_vis[floor_num])
			{
				bot->start_tile_floor[floor_num] = bot->index;
				bot->floors_vis[floor_num] = true;
			}
		}			
		else if(down_ramp)
		{
			set_ramp_wall();
			int up_ramp_index = sim::get_up_ramp_index(sim::sim_robot_index);
			if(up_ramp_index == -1)
			{
				std::cerr << "ERR: no downramp found!" << std::endl;
				return false;
			}
			bot->map[bot->index].bot = false;
			nodes[sim::sim_robot_index].bot = false;
			bot->map[bot->index].ramp = 0b10;
			floor_num--;
			bot->map = bot->floors[floor_num];
			nodes = floors[floor_num - start_floor];
			bot->index += ramp_len * delta;
			sim::sim_robot_index = up_ramp_index;
			bot->map[bot->index].bot = true;
			nodes[sim::sim_robot_index].bot = true;
			bot->map[bot->index - delta].ramp = 0b01;

			if(!bot->floors_vis[floor_num])
			{
				bot->start_tile_floor[floor_num] = bot->index;
				bot->floors_vis[floor_num] = true;
			}
		}
		
		return true;
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
		switch(bot->dir)
		{
			case DIR::N:
			{
				if(helper::is_valid_index(bot->index - horz_size) && helper::is_valid_index(sim::sim_robot_index - horz_size) && !bot->map[bot->index].N)
				{
					if(!set_mov_indexes(-horz_size))
					{
						return false;
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
					if(!set_mov_indexes(1))
					{
						return false;
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
					if(!set_mov_indexes(horz_size))
					{
						return false;
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
					if(!set_mov_indexes(-1))
					{
						return false;
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

	CREATE_DRIVER(void, save_state)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		std::ofstream out;
		out.open("save.txt");
		std::cout << "saving state" << std::endl;
		//out << helper::dir_to_char(bot->dir) << std::endl;
		for(int l = 0; l < max_num_floors; l++)
		for(int i = 0; i < horz_size * vert_size; i++)
		{
			node n = bot->floors[l][i];
			out << n.N << " " << n.E << " " << n.S << " " << n.W << " ";
			out << n.vic << " " << n.bot << " " << n.vis << " " << n.ramp << " ";
			out << n.checkpoint << std::endl;
		}
	}

	CREATE_DRIVER(void, load_state)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		std::ifstream in;
		in.open("save.txt");
		bool n, e, s, w, vic, vis, checkpoint, bot_here;
		uint8_t ramp;
		//char dir;
		//in >> dir;
		//wtf happens with directions, I think we can only restart the raspberry pi
		//bot->dir = helper::char_to_dir(dir);
		for(int l = 0; l < max_num_floors; l++)
		for(int i = 0; i < horz_size * vert_size; i++)
		{
			in >> n >> e >> s >> w >> vic >> bot_here >> vis >> ramp >> checkpoint;
			if(bot_here)
			{
				bot->index = i;
				bot->map = bot->floors[l];
				floor_num = l;
			}
			
			node& temp = bot->floors[l][i];
			temp.N = n;
			temp.E = e;
			temp.S = s;
			temp.W = w;
			temp.vic = vic;
			temp.vis = vis;
			temp.ramp = ramp;
			temp.checkpoint = checkpoint;
			temp.bot = bot_here;
		}
		std::cout << "loaded save file!" << std::endl;
	}

	CREATE_DRIVER(void, init_robot)
	{
		CHECK(robot::get_instance());
		PythonScript::initPython();
		PythonScript::Exec(init_py_file);
		
		if(std::filesystem::exists("save.txt"))
			load_state();
	}

	CREATE_DRIVER(void, cleanup)
	{
		PythonScript::Exec(cleanup_py_file);
	}

	CREATE_DRIVER(void, drop_vic, int num, bool left)
	{
		//d [drop] N [number of kits, single digit only, can be 0 which will trigger the led] l/r [direction] \n
		std::string direction = left ? "l" : "r";
		Bridge::remove_data_value("drop_status");
		PythonScript::CallPythonFunction<bool, std::string>("SendSerialCommand", com::drop_vic + std::to_string(num) + direction + "\n");
		
		while(!Bridge::get_data_value("drop_status").has_value())
		{
			PythonScript::Exec(ser_py_file);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		
		while((bool)(*Bridge::get_data_value("drop_status"))[0])
		{
			PythonScript::Exec(ser_py_file);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
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
			if(bot->map[bot->index].checkpoint)
				save_state();
		}
		//debug::print_node(bot->map[bot->index]);
		
		Bridge::remove_data_value("victim");
		Bridge::remove_data_value("NRK");
		Bridge::remove_data_value("left");
	
		if(!bot->map[bot->index].vic)
		{
			for(int i = 0; i < 2; i++) 
			{
				PythonScript::Exec(cv_py_file);
				bool victim = (bool)(*Bridge::get_data_value("victim"))[0];
				bool left = (bool)(*Bridge::get_data_value("left"))[0];
				int num_rescue = (int)(*Bridge::get_data_value("NRK"))[0];
				if(victim)
				{
					drop_vic(num_rescue, left);
					bot->map[bot->index].vic = true;
				}
			}
		}
	}

	void set_mov_indexes(int delta, bool up_ramp, bool down_ramp, int ramp_len)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		int org_index = bot->index;

		bot->map[bot->index].bot = false;
		bot->index += delta;
		if(!down_ramp && !up_ramp && bot->map[org_index].vic)
		{
			bot->map[bot->index].vic = true;
		}
		bot->map[bot->index].bot = true;

		if(up_ramp)
		{
			set_ramp_wall();
			bot->map[bot->index].bot = false;
            bot->map[bot->index].ramp = 0b01;
			floor_num++;
			bot->map = bot->floors[floor_num];

			bot->index += delta * ramp_len;

			bot->map[bot->index - delta].ramp = 0b10;
			bot->map[bot->index - delta].vis = true;
			bot->map[bot->index].bot = true;

			if(!bot->floors_vis[floor_num])
			{
				bot->start_tile_floor[floor_num] = bot->index - delta;
				bot->floors_vis[floor_num] = true;
			}

			std::cout << "on a higher floor: " << floor_num << ',' << bot->index << std::endl;
		}
		else if(down_ramp)
		{
			set_ramp_wall();
			bot->map[bot->index].bot = false;
            bot->map[bot->index].ramp = 0b10;
			floor_num--;
			bot->map = bot->floors[floor_num];

			bot->index += delta * ramp_len;

			bot->map[bot->index - delta].ramp = 0b01;
			bot->map[bot->index - delta].vis = true;
			bot->map[bot->index].bot = true;

			if(!bot->floors_vis[floor_num])
			{
				bot->start_tile_floor[floor_num] = bot->index - delta;
				bot->floors_vis[floor_num] = true;
			}

			std::cout << "on a lower floor: " << floor_num << ',' << bot->index << std::endl;
		}

		Bridge::remove_data_value("ramp");
	}

	CREATE_DRIVER(bool, forward)
	{
		robot* bot = robot::get_instance();
		CHECK(bot);
		CHECK(bot->map);
		auto org_index = bot->index;
		auto org_floor = floor_num;
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
		
		Bridge::remove_data_value("victim");
		
		//wait for it to finish running
		auto time_step = std::chrono::high_resolution_clock::now();
		while((bool)(*Bridge::get_data_value("forward_status"))[0]) 
		{ 
			PythonScript::Exec(ser_py_file);
			PythonScript::Exec(cv_py_file);
			if(!bot->map[bot->index].vic && std::chrono::high_resolution_clock::now() - time_step >= std::chrono::milliseconds(600))
			{
				for(int i = 0; i < 2; i++)
				{
					PythonScript::Exec(cv_py_file);
					bool victim = (*Bridge::get_data_value("victim"))[0];
					bool left = (*Bridge::get_data_value("left"))[0];
					int nrk = (*Bridge::get_data_value("NRK"))[0];
					if(victim)
					{
						drop_vic(nrk, left);
						bot->map[bot->index].vic = true;
					}
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		int ramp = (int)(*Bridge::get_data_value("ramp"))[0];
		int ramp_len = (int)(*Bridge::get_data_value("ramp"))[1];
		//works since it can't be both :)
		bool up_ramp = ramp == 1;
		bool down_ramp = ramp == 10;
		if(up_ramp || down_ramp)
		{
			std::cerr << "ramp recv: " << ramp << "," << ramp_len << std::endl;
		}

		switch(bot->dir)
		{
			case DIR::N:
			{
				if(helper::is_valid_index(bot->index - horz_size) && !bot->map[bot->index].N){
					set_mov_indexes(-horz_size, up_ramp, down_ramp, ramp_len);
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
					set_mov_indexes(1, up_ramp, down_ramp, ramp_len);
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
					set_mov_indexes(horz_size, up_ramp, down_ramp, ramp_len);
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
					set_mov_indexes(-1, up_ramp, down_ramp, ramp_len);
				}
				else{
					std::cerr << "ERROR: Cannot move forward!" << std::endl;
					debug::print_robot_info(bot);
					return false;
				}
				break;
			}
		}

		auto status = *Bridge::get_data_value("forward_status");
		bool black_tile = !(bool)status[1];
		bool failed = (bool)status[2];

		//std::cout << "status recv: " << black_tile << "," << failed << std::endl;

		//black_tile = true when black tile (since we negated above)
		//failed = true when failed
		if(failed || black_tile)
		{
			floor_num = org_floor;
			bot->map = bot->floors[org_floor];
			bot->map[org_index].bot = true;
			bot->map[bot->index].bot = false;
			if(black_tile)
			{
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
				
				std::cout << "driver::forward: WARN: Black tile detected, returning false" << std::endl;
			}
			else
			{
				std::cout << "drive::forward: WARN: Failed to move forward, returning false" << std::endl;
				bot->index = org_index;
				floor_num = org_floor;
				bot->map = bot->floors[org_floor];
				bot->map[bot->index].vis = false;
				get_sensor_data();
			}

			bot->index = org_index;

			//don't read wall data only when black tile
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
		
		Bridge::remove_data_value("turn_status");
		PythonScript::CallPythonFunction<bool, std::string>("SendSerialCommand", turn_cmd);

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
