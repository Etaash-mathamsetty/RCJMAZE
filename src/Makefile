OBJS = BFS.o robot.o simulation.o helpers.o driver.o debug_helper.o scripting.o
CFLAGS += -Wall -O2 -g -std=c++17 `pkg-config --cflags python3`

LDFLAGS = -lboost_python3 -lpython3.10

all:	BFS

BFS:	$(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o:	%.cpp
	$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ $<

%.o:	%.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -rf BFS $(OBJS)
