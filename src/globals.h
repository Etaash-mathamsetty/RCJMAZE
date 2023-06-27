#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED

#include <inttypes.h>
#include <assert.h>
#include <sstream>
#include <utility>

// #define SIMULATION
#define DEBUG
//#define SIM_MOV_DELAY

#ifdef SIMULATION
//is the bigger one in terms of size, shouldn't impact the smaller node's values when using memcpy
struct simulation_node {
	public:
		bool N : 1;
		bool S : 1;
		bool E : 1;
		bool W : 1;
		bool vic : 1;
		bool bot : 1;
		bool vis : 1;
		//ramp up is first bit, ramp down is second bit
		uint8_t ramp : 2;
		bool checkpoint : 1;
		bool black : 1;
	private:
		uint8_t garbage : 5;

};
#endif

// is the smaller in terms bytes
struct node {
	public:
#ifdef SIMULATION
		node(const simulation_node& node)
		{
			N = node.N;
			S = node.S;
			E = node.E;
			W = node.W;
			vic = node.vic;
			bot = node.bot;
			vis = node.vis;
			ramp = node.ramp;
			checkpoint = node.checkpoint;
		}

		node() {}
#endif

		bool N : 1;
		bool S : 1;
		bool E : 1;
		bool W : 1;
		bool vic : 1; //have we already seen a victim on this tile
		bool bot : 1;
		bool vis : 1;
		uint8_t ramp : 2;
		bool checkpoint : 1;
	private:
		uint8_t garbage : 6;
};

enum class DIR
{
		N,
		E,
		S,
		W
};

struct nearest_quad
{
	int nearest[4];

	int& operator[] (int x)
	{
		return nearest[x % 4];
	}

	explicit operator std::string()
	{
		std::stringstream x;
		x << "E: " << nearest[0] << ", W: " << nearest[1] << ", S: " << nearest[2] << ", N: " << nearest[3];
		return x.str();
	}
};

const int horz_size = 80;
const int vert_size = horz_size;
const int default_index = (horz_size/2) - 1;

namespace com
{
	const char turn = 't';
	const char go = 'g';
	const char forward = 'f';
	const char east = 'e';
	const char west = 'w';
	const char north = 'n';
	const char south = 's';
	const char quit = 'q';
	const char drop_vic = 'd';
};

const std::string init_py_file = "init.py";
const std::string cv_py_file = "cv.py";
const std::string ser_py_file = "serial.py";
const std::string cleanup_py_file = "cleanup.py";

const int max_num_floors = 10;
const int start_floor = 4;
inline int floor_num = start_floor;

#ifdef SIMULATION
//simulation field
/* number of simulation_node* we need to allocate for second_floor */
inline int num_floors = 1;
inline simulation_node** floors;
inline simulation_node* nodes;
const static bool is_simulation = true;
#else
const static bool is_simulation = false;
#endif

#define CHECK(ptr) assert(ptr)

#endif // GLOBALS_H_INCLUDED
