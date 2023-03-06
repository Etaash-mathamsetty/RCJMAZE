OBJS = BFS.o robot.o simulation.o helpers.o driver.o debug.o scripting.o
CFLAGS += -Wall -O2 -g -std=c++17 `python-config --cflags`

LDFLAGS = -lboost_python3 `python-config --ldflags`

all:	BFS

BFS:	$(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o:	%.cpp
	$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ $<

%.o:	%.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -rf BFS $(OBJS)
