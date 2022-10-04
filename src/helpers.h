#include "globals.h"
#include <string>

#ifndef __HELPERS_H__
#define __HELPERS_H__
namespace helper
{
    int get_index(const int& a, const int& b);

    bool is_valid_index(const int& index);

    std::string dir_to_string(const DIR& dir);

    char dir_to_char(const DIR& dir);

    DIR char_to_dir(const char& c);

    DIR prev_dir(const DIR& dir);

    DIR next_dir(const DIR& dir);
};

#endif