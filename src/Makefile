

OBJS = BFS.o robot.o simulation.o helpers.o driver.o debug.o scripting.o
CFLAGS = -Wall -std=c++17 `pkg-config --cflags python3`

LDFLAGS = -lboost_python3 -lpython3.10

all:	run

run:	CFLAGS += -O2
run:	BFS

debug:	CFLAGS += -g
debug:	BFS

BFS:	$(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	$(EXTRA_CMDS)

%.o:	%.cpp
	$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $<

%.o:	%.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -rf BFS $(OBJS)
