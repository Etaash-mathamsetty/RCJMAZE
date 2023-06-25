#include "debug.h"
#include <fstream>

static std::string last_error = "No Error";

namespace debug
{
    void print_node(const node& node)
	{
		#ifdef DEBUG
			printf("N: %d S: %d E: %d W: %d vic: %d bot: %d vis: %d\n", node.N, node.S, node.E, node.W, node.vic, node.bot, node.vis);
		#endif
	}

    void print_robot_info(robot* bot)
	{
		#ifdef DEBUG
		CHECK(bot);
		std::cout << "DEBUG INFO: " << std::endl;
		std::cout << (std::string)(*bot) << std::endl;
		print_node(bot->map[bot->index]);
		#endif
	}   

    void print_map()
	{
#ifdef DEBUG
#ifdef SIMULATION
        for(int i = 0; i < sim::_vert_size[floor_num - start_floor]; i++)
		{
            for(int l = 0; l < sim::_horz_size[floor_num - start_floor]; l++)
			{
                putchar('+');
                if(nodes[helper::get_index(i,l)].N)
                    putchar('-');
                else
                    putchar(' ');
            }
            putchar('\n');
            for(int l = 0; l < sim::_horz_size[floor_num - start_floor]; l++)
			{
                if(nodes[helper::get_index(i,l)].W)
                    putchar('|');
                else
                    putchar(' ');
                if(nodes[helper::get_index(i,l)].bot)
                    putchar('x');
                if(nodes[helper::get_index(i,l)].vic)
                    putchar('v');
                if(nodes[helper::get_index(i,l)].checkpoint)
                    putchar('c');
                if(nodes[helper::get_index(i,l)].black)
                    putchar('b');
                if(nodes[helper::get_index(i, l)].ramp)
                    putchar('r');
                if(!nodes[helper::get_index(i,l)].bot && !nodes[helper::get_index(i,l)].vic && !nodes[helper::get_index(i,l)].checkpoint &&
                    !nodes[helper::get_index(i,l)].black && !nodes[helper::get_index(i, l)].ramp)
                    putchar(' ');
            }
            putchar('\n');
        }
        for(int l = 0; l < sim::_horz_size[floor_num - start_floor]; l++)
		{
            putchar('+');
            if(nodes[helper::get_index(sim::_vert_size[floor_num - start_floor]-1,l)].S)
                putchar('-');
            else
                putchar(' ');
        }
        putchar('\n');
#else
    robot* bot = robot::get_instance();
    CHECK(bot);
    CHECK(bot->map);
    std::cout << "TODO: Print more than current tile!\n" << std::endl;
    node cur_node = bot->map[bot->index];
    putchar('+');
    putchar(cur_node.N ? '-' : ' ');
    puts("+");
    putchar(cur_node.W ? '|' : ' ');
    if(cur_node.vic)
        putchar('v');
    if(cur_node.checkpoint)
        putchar('c');
    if(cur_node.ramp)
        putchar('r');
    if(!cur_node.vic && !cur_node.checkpoint && !cur_node.ramp)
        putchar(' ');
    putchar(cur_node.E ? '|' : ' ');
    putchar('\n');
    putchar('+');
    putchar(cur_node.S ? '-' : ' ');
    puts("+\n");
    
    std::ofstream out("debug_field.txt");
    node* nodes = bot->map; 
    
    for(int i = 0; i < vert_size; i++)
		{
            for(int l = 0; l < horz_size; l++)
			{
                out << '+';
                if(nodes[helper::get_index(i,l)].N)
                    out << '-';
                else
                    out << ' ';
            }
            out << '\n';
            for(int l = 0; l < horz_size; l++)
			{
                if(nodes[helper::get_index(i,l)].W)
                    out << '|';
                else
                    out << ' ';
                if(nodes[helper::get_index(i,l)].bot)
                    out << 'x';
                if(nodes[helper::get_index(i,l)].vic)
                    out << 'v';
                if(nodes[helper::get_index(i,l)].checkpoint)
                    out << 'c';
                if(nodes[helper::get_index(i, l)].ramp)
                    out << 'r';
                if(!nodes[helper::get_index(i,l)].bot && !nodes[helper::get_index(i,l)].vic && !nodes[helper::get_index(i,l)].checkpoint && !nodes[helper::get_index(i, l)].ramp)
                    out << ' ';
            }
            out << '\n';
        }
        for(int l = 0; l < horz_size; l++)
		{
            out << '+';
            if(nodes[helper::get_index(vert_size-1,l)].S)
                out << '-';
            else
                out << ' ';
        }
        out << '\n';
#endif
#endif
    }

    void print_path(Stack<int>& path)
    {
        #ifdef DEBUG
        for(size_t i = 0; i < path.Size(); i++)
        {
            if(i > 0)
                putchar(',');
            std::cout << path[i];
        }
        putchar('\n');
        #endif
    }

    //will use for debugging
    void set_last_error(const std::string& error)
    {
        last_error = error;
    }

    const std::string& get_last_error()
    {
        return last_error;
    }
}
