#include <inttypes.h>
#include <assert.h>
#include <sstream>

#ifndef _GLOBALS
#define _GLOBALS

#define SIMULATION
#define DEBUG
#define SIM_MOV_DELAY

/* TODO: lack of progress support
	save state and load state for checkpoints
*/

struct node{
	public:
		bool N : 1;
		bool S : 1;
		bool E : 1;
		bool W : 1;
		bool vic : 1; //have we already seen a victim on this tile
		bool bot : 1;
		bool vis : 1;
		bool ramp : 1;
		bool checkpoint : 1;
	private:
		uint8_t garbage : 7;
};

enum class DIR{
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

const int default_index = 19;
const int horz_size = 40;
const int vert_size = 40;

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
};

const std::string init_py_file = "vars.py";
const std::string cv_py_file = "cv.py";
const std::string ser_py_file = "serial.py";
const std::string cleanup_py_file = "cleanup.py";

#ifdef SIMULATION
//simulation field
inline node* nodes;
const static bool is_simulation = true;
#else
const static bool is_simulation = false;
#endif

#define CHECK(ptr) assert(ptr)

#endif
