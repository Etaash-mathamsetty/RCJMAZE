#include "globals.h"

#ifndef __HELPERS_H__
#define __HELPERS_H__
namespace helper
{

	inline int get_index(int a, int b){
		return a * horz_size + b;
	}

	inline bool is_valid_index(int index) {
		return index < (horz_size * vert_size) && index > 0;
	}

    inline std::string dir_to_string(const DIR& dir){
        switch(dir){
            case DIR::N:
                return "North";
            case DIR::E:
                return "East";
            case DIR::S:
                return "South";
            case DIR::W:
                return "West";
            default:
                return "Invalid Direction";
        }
    }

};

#endif