CXX = g++
CXX_FLAGS = -Wall -pedantic -std=c++11 -c
LINK_PATH = -L ../common/

VPATH = src \
		obj

MOTION_CONTROLLER_SRCS = main.cpp
MOTION_CONTROLLER_OBJS = $(MOTION_CONTROLLER_SRCS:.cpp=.o)

motion_controller: obj/main.o $(MOTION_CONTROLLER_OBJS:%=obj/%)
	$(CXX) $(LINK_PATH) $^ -o $@ -l common

clean:
	rm obj/*.o
	rm motion_controller

obj/main.o: main.cpp
	$(CXX) $(CXX_FLAGS) $< -o $@

obj/%.o: %.cpp %.hpp
	$(CXX) $(CXX_FLAGS) $< -o $@
