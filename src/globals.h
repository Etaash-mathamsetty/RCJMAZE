#include <inttypes.h>
#include <assert.h>
#include <sstream>

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
		bool vic : 1; //TODO: use more bits for this to represent different victims and multiple victims
		bool bot : 1;
		bool vis : 1;
	private:
		uint8_t garbage : 1;
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
		return nearest[x];
	}

	explicit operator std::string()
	{
		std::stringstream x;
		x << "E: " << nearest[0] << ", W: " << nearest[1] << ", S: " << nearest[2] << ", N: " << nearest[3];
		return x.str();
	}
};

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

#define CHECK(ptr) assert(ptr)

#endif
