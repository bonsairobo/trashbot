CXX = g++
CXX_FLAGS = -Wall -pedantic -std=c++11 -c -g
INCLUDE_PATH = -I libfreenect/include -I OpenNI2/Include
LINK_PATH = -L libfreenect/build/lib -L libfreenect/build/lib/OpenNI2-FreenectDriver -L OpenNI2/Bin/x64-Release

VPATH = src \
		obj

KINECT_SRCS = main.cpp

KINECT_OBJS = $(KINECT_SRCS:.cpp=.o)

kinect: obj/main.o $(KINECT_OBJS:%=obj/%)
	$(CXX) $(LINK_PATH) $^ -o $@ -fPIC -l freenect -l OpenNI2

clean:
	rm obj/*.o
	rm kinect

obj/main.o: main.cpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@

obj/%.o: %.cpp %.hpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@
