#include <inttypes.h>
#include <assert.h>

#ifndef _GLOBALS
#define _GLOBALS

#define SIMULATION
#define DEBUG

struct node{
	public:
		bool N : 1;
		bool S : 1;
		bool E : 1;
		bool W : 1;
		bool vic : 1;
		bool bot : 1;
		uint8_t vis : 2;
};

enum class DIR{
		N,
		E,
		S,
		W
};

static std::string dir_to_string(DIR dir){
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


inline void print_node(node node){
	printf("N: %d S: %d E: %d W: %d vic: %d bot: %d vis: %d\n", node.N, node.S, node.E, node.W, node.vic, node.bot, node.vis);
}



#ifdef SIMULATION
inline int horz_size;
inline int vert_size;
inline node* nodes;
const static bool is_simulation = true;
#else
inline int horz_size = 40;
inline int vert_size = 40;
const static bool is_simulation = false;
#endif


inline int get_index(int a, int b){
	return a * horz_size + b;
}

inline bool is_valid_index(int index) {
	return index < (horz_size * vert_size) && index > 0;
}

#define CHECK(ptr) assert(bot)

#endif
