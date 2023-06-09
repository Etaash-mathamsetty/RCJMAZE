#include "simulation.h"
#include "driver.h"
#include <algorithm>
#include <utility>
#include <filesystem>

namespace sim
{
#ifdef SIMULATION
    inline void set_node_values(simulation_node& node, char c)
    {
        char _c = tolower(c);
        node.bot |= (_c == 'x');
        node.vis |= node.bot;
        node.vic |= (_c == 'v');
        node.checkpoint |= (_c == 'c');
        node.black |= (_c == 'b');
        //using & operator just in case
        node.ramp |= (_c == 'u') & 0b1;
        node.ramp |= ((_c == 'd') << 1) & 0b10;
    }

    int floor_number = 0;

    void read_map_from_file(std::string name)
    {
        std::ifstream in(name);
        if(in.fail())
        {
            std::cerr << "ERR: Cannot open " << name << std::endl;
            return;
        }
        //std::cout << "sizeof DIR: " << sizeof(DIR) << std::endl;
        char x = 0;
        in >> _horz_size;
        _horz_size++;
        in >> _vert_size;

        std::vector<int>* ramps_cur_floor_pos = (ramp_positions + floor_number); 
        floors[floor_number] = new simulation_node[horz_size * vert_size];
        memset(floors[floor_number], 0, sizeof(simulation_node) * horz_size * vert_size);
        nodes = floors[floor_number];

        if(std::filesystem::exists("save.txt"))
            return;

        //int v = 0;
        in.get(x);
        for(int v = 0; v < _vert_size; v++)
        {
            if(v > 0){
                for(int i = 0; i < _horz_size; i++){
                    //probably wont be necessary on an actual robot, but we need it to read from the field.txt properly
                    nodes[helper::get_index(v,i)].N = nodes[helper::get_index(v-1,i)].S;
                }
            }
            //printf("loop 1\n");
            //get rid of extra plus
            //in.get(x);
            if(v == 0)
                for(float i = 0; i < _horz_size; i+=0.5)
                {
                    in.get(x);
                    //printf("x: %c i: %d\n", x, i);
                    if(x == '\0')
                        return;
                    if(x == '\n')
                        break;
                    if(x == '+') continue;
                    auto& node = nodes[helper::get_index(v,i)];
                    node.N = (x == '-');
                    set_node_values(node, x);

                    if(node.bot)
                        sim_robot_index = helper::get_index(v,i);
                    
                    if(node.ramp)
                        ramps_cur_floor_pos->push_back(helper::get_index(v,i));
                    //print_node(nodes[get_index(v,i)]);
                    //nodes[get_index(v,i)] = node;
                    //horz_size++;
                }
            //printf("loop 2\n");
            for(float i = 0; i < _horz_size; i+=0.5)
            {
                in.get(x);
                //printf("x: %c\n", x);
                if(x == '\0')
                    return;
                if(x == '\n')
                    break;
                //if(x == ' ') { i--; continue; }
                auto& node = nodes[helper::get_index(v,i)];
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
                set_node_values(node, x);

                if(node.bot)
                    sim_robot_index = helper::get_index(v,i);

                if(node.ramp)
                    ramps_cur_floor_pos->push_back(helper::get_index(v,i));
            }
            //printf("loop 3\n");
            for(float i = 0; i < _horz_size; i+=0.5)
            {
                in.get(x);
                //printf("x: %c\n", x);
                if(x == '\0')
                    return;
                if(x == '\n')
                    break;
                if(x == '+') continue;
                auto& node = nodes[helper::get_index(v,i)];
                node.S = (x == '-');
                set_node_values(node, x);

                if(node.bot)
                    sim_robot_index = helper::get_index(v,i);
                
                if(node.ramp)
                    ramps_cur_floor_pos->push_back(helper::get_index(v,i));
            }
            //v++;
        }
        //create ramp pairs here: (create pairs, not on the first floor)
        if(floor_number >= 1)
        {
            std::string map_key = std::to_string(floor_number) + "," + std::to_string(floor_number-1);
            ramp_pair pair;
            for(int i = 0; i < ramps_cur_floor_pos->size(); i++)
            {
                //recheck this logic
                if(floors[floor_number][ramp_positions[floor_num][i]].ramp & 0b1 && floors[floor_num - 1][ramp_positions[floor_num - 1][i]].ramp & 0b10)
                {
                    pair.ramp1 = std::make_pair(floor_num - 1, ramp_positions[floor_num - 1][i]);
                    pair.ramp2 = std::make_pair(floor_num, ramp_positions[floor_num][i]);
                    ramp_pairs[map_key].push_back(pair);
                }
            }
        }
        
        floor_number++;
        num_floors = floor_number;
        return;
    }

    ramp_pair* get_ramp_pair(int floor_num, int index)
    {
        std::string map_key = std::to_string(floor_number) + "," + std::to_string(floor_number - 1);
        ramp_pair pair;
        for(int i = 0; i < ramp_pairs[map_key].size(); i++)
        {
            if(ramp_pairs[map_key][i].ramp1.second == index)
            {
                return &ramp_pairs[map_key][i];
            }
        }

        return nullptr;
    }

    //vim style commands ig?
    bool run_command()
    {
        robot* bot = robot::get_instance();
        CHECK(bot);
        std::cout << "Enter a command:";
        std::string input, cur_cmd = "";
        std::cin >> input;
        for(char& c : input) { c = std::tolower(c); }
        if(input == "help")
        {
            std::cout << "INFO: I was too lazy to write a help section, so ask Etaash or just go read the source code." << std::endl;
            return true;
        }
        input += '\n';
        for(char c : input)
        {
            if(c == com::quit)
                return false;
            if(c == 'c')
                system("clear");
            if(c == com::go || c == com::forward || c == com::forward)
            {
                if(cur_cmd.length() > 0)
                {
                    if(cur_cmd[0] == com::go || cur_cmd[0] == com::forward)
                    {
                        bot->forward();
                    }
                    else
                    {
                        std::cerr << "ERR: Invalid Parameter" << std::endl;
                    }
                }
                cur_cmd.clear();
                cur_cmd += c;
                continue;
            }
            if(c == com::east || c == com::west || c == com::south || c == com::north)
            {
                if(cur_cmd.length() > 0)
                {
                    if(cur_cmd[0] == com::forward || cur_cmd[0] == com::go)
                    {
                        driver::turn_to(helper::char_to_dir(c));
                        bot->forward();
                    }
                    else if(cur_cmd[0] == com::turn)
                    {
                        driver::turn_to(helper::char_to_dir(c));
                    }
                    else
                    {
                        std::cerr << "ERR: Invalid Command" << std::endl;
                    }
                    cur_cmd.clear();
                    continue;
                }
                else
                {
                    std::cerr << "ERR: Invalid Command" << std::endl;
                    continue;
                }
            }
            if(c == '\n' || c == '\0')
            {
                if(cur_cmd.length() > 0)
                {
                    if(cur_cmd[0] == com::go || cur_cmd[0] == com::forward)
                    {
                        bot->forward();
                    }
                    else
                    {
                        std::cerr << "ERR: Invalid Parameter" << std::endl;
                    }
                }
                cur_cmd.clear();
                continue;
            }
        }
        return true;
    }
#endif
};
