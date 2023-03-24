#include "simulation.h"
#include "driver.h"
#include <algorithm>
#include <filesystem>

namespace sim
{

    inline void set_node_values(simulation_node& node, char c)
    {
        node.bot |= (tolower(c) == 'x');
        node.vis |= node.bot;
        node.vic |= (tolower(c) == 'v');
        node.checkpoint |= (tolower(c) == 'c');
        node.black |= (tolower(c) == 'b');
        node.ramp |= (tolower(c) == 'r');
    }

    int floor_number = 0;

    #ifdef SIMULATION
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

        //init with a random value
        int& _second_floor_entrance = floor_number;

        if(floor_number != 0)
        {
            _second_floor_entrance = second_floor_entrance[floor_number - 1];
            nodes = second_floor[floor_number - 1];
        }

        floor_number++;

        nodes = new simulation_node[horz_size * vert_size];
        memset(nodes, 0, sizeof(simulation_node) * horz_size * vert_size);

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
                    
                    if(node.ramp && floor_number != 0)
                        _second_floor_entrance = helper::get_index(v,i);
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

                if(node.ramp && floor_number != 0)
                    _second_floor_entrance = helper::get_index(v,i);
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
                
                if(node.ramp && floor_number != 0)
                    _second_floor_entrance = helper::get_index(v,i);
            }
            //v++;
        }
        return;
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
