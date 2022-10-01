

OBJS = BFS.o robot.o simulation.o
CFLAGS = -Wall -std=c++17

LDFLAGS =

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
