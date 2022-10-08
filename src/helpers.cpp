#include "helpers.h"
#include "globals.h"
#include "robot.h"

namespace helper
{
    int get_index(const int& a, const int& b){
		return a * horz_size + b;
	}

    bool is_valid_index(const int& index) {
		return index < (horz_size * vert_size) && index >= 0;
	}

    std::string dir_to_string(const DIR& dir){
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

    char dir_to_char(const DIR& dir)
    {
        switch(dir)
        {
            case DIR::N:
                return 'n';
            case DIR::E:
                return 'e';
            case DIR::S:
                return 's';
            case DIR::W:
                return 'w';
            default:
                return 'n';
        }
    }

    DIR char_to_dir(const char& c)
    {
        switch(tolower(c))
        {
            case 'n':
                return DIR::N;
            case 'e':
                return DIR::E;
            case 's':
                return DIR::S;
            case 'w':
                return DIR::W;
            default: 
                return DIR::N;
        }
    }

    DIR prev_dir(const DIR& dir)
    {
        if(dir == DIR::N)
			return DIR::W;
		else
			//can't do dir-- so...
			return DIR(int(dir)-1);
    }

    DIR next_dir(const DIR& dir)
    {
        if(dir == DIR::W)
			return DIR::N;
		else
			//can't do dir++ so...
			return DIR(int(dir)+1);
    }

    nearest_quad get_nearest(const int& _index)
    {
        nearest_quad quad;
        node* map = robot::get_instance()->map;
        //std::cout << ((is_valid_index((int)index-1) && !map[index].W) ? index-1 : 0u) << std::endl;
        quad[0] = ((is_valid_index(_index+1) && !map[_index].E) ? _index+1 : -1);
        quad[1] = ((is_valid_index(_index-1) && !map[_index].W) ? _index-1 : -1);
        quad[2] = ((is_valid_index(_index+horz_size) && !map[_index].S) ? _index+horz_size : -1);
        quad[3] = ((is_valid_index(_index-horz_size) && !map[_index].N) ? _index-horz_size : -1);
        return quad;
    }

} // namespace helper
