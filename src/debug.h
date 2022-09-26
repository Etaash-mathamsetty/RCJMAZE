#include "robot.h"
#include "helpers.h"
#include "globals.h"
#include <iostream>

#ifndef __DEBUG_H__
#define __DEBUG_H__

namespace debug
{

	inline void print_node(const node& node)
	{
		#ifdef DEBUG
			printf("N: %d S: %d E: %d W: %d vic: %d bot: %d vis: %d\n", node.N, node.S, node.E, node.W, node.vic, node.bot, node.vis);
		#endif
	}

	inline void print_robot_info(robot* bot)
	{
		#ifdef DEBUG
		CHECK(bot);
		std::cout << "DEBUG INFO: " << std::endl;
		std::cout << (std::string)(*bot) << std::endl;
		print_node(bot->map[bot->index]);
		#endif
	}

	inline void print_map()
	{
		#ifdef DEBUG
        for(int i = 0; i < vert_size; i++)
		{
            for(int l = 0; l < horz_size; l++)
			{
                putchar('+');
                if(nodes[helper::get_index(i,l)].N)
                    putchar('-');
                else
                    putchar(' ');
            }
            putchar('\n');
            for(int l = 0; l < horz_size; l++)
			{
                if(nodes[helper::get_index(i,l)].W)
                    putchar('|');
                else
                    putchar(' ');
                if(nodes[helper::get_index(i,l)].bot)
                    putchar('x');
                if(nodes[helper::get_index(i,l)].vic)
                    putchar('v');
                if(!nodes[helper::get_index(i,l)].bot && !nodes[helper::get_index(i,l)].vic)
                    putchar(' ');
            }
            putchar('\n');
        }
        for(int l = 0; l < horz_size; l++)
		{
            putchar('+');
            if(nodes[helper::get_index(vert_size-1,l)].S)
                putchar('-');
            else
                putchar(' ');
        }
        putchar('\n');
		#endif
    }
};

#endif