CXX = g++
CXX_FLAGS = -Wall -pedantic -std=c++11 -c
LINK_PATH = -L ../common/

VPATH = src \
		obj

JOYSTICK_SRCS = main.cpp joystick.cpp

JOYSTICK_OBJS = $(JOYSTICK_SRCS:.cpp=.o)

joystick: obj/main.o $(JOYSTICK_OBJS:%=obj/%)
	$(CXX) $(LINK_PATH) $^ -o $@ -l common

clean:
	rm obj/*.o
	rm joystick

obj/main.o: main.cpp
	$(CXX) $(CXX_FLAGS) $< -o $@

obj/%.o: %.cpp %.hpp
	$(CXX) $(CXX_FLAGS) $< -o $@
