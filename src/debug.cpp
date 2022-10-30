#include "debug.h"

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
        for(int i = 0; i < sim::_vert_size; i++)
		{
            for(int l = 0; l < sim::_horz_size; l++)
			{
                putchar('+');
                if(nodes[helper::get_index(i,l)].N)
                    putchar('-');
                else
                    putchar(' ');
            }
            putchar('\n');
            for(int l = 0; l < sim::_horz_size; l++)
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
                if(!nodes[helper::get_index(i,l)].bot && !nodes[helper::get_index(i,l)].vic && !nodes[helper::get_index(i,l)].checkpoint)
                    putchar(' ');
            }
            putchar('\n');
        }
        for(int l = 0; l < sim::_horz_size; l++)
		{
            putchar('+');
            if(nodes[helper::get_index(sim::_vert_size-1,l)].S)
                putchar('-');
            else
                putchar(' ');
        }
        putchar('\n');
        #else
        std::cout << "FIXME: unimplemented print_map()" << std::endl;
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