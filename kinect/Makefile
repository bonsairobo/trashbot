CXX = g++
CXX_FLAGS = -Wall -pedantic -std=c++11 -c -g

VPATH = src \
		obj

KINECT_SRCS = main.cpp

KINECT_OBJS = $(COMPASS_SRCS:.cpp=.o)

kinect: obj/main.o $(KINECT_OBJS:%=obj/%)
	$(CXX) $^ -o $@ -fPIC -l freenect

clean:
	rm obj/*.o
	rm kinect

obj/main.o: main.cpp
	$(CXX) $(CXX_FLAGS) $< -o $@

obj/%.o: %.cpp %.hpp
	$(CXX) $(CXX_FLAGS) $< -o $@
