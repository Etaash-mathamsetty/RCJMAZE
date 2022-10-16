#include "globals.h"
#include <string>

#ifndef __HELPERS_H__
#define __HELPERS_H__
namespace helper
{
    // returns a 1d index from a 2d index
    int get_index(const int&, const int&);

    bool is_valid_index(const int&);

    std::string dir_to_string(const DIR&);

    char dir_to_char(const DIR&);

    DIR char_to_dir(const char&);

    DIR prev_dir(const DIR&);

    DIR next_dir(const DIR&);

    //get nearest tile that is reachable from the current tile
    nearest_quad get_nearest(const int&);
};

#endif