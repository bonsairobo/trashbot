CXX = g++
CXX_FLAGS = -Wall -pedantic -std=c++11 -c
INCLUDE_PATH = -I libfreenect/include -I OpenNI2/Include
LINK_PATH = -L opencv/build/lib \
			-L libfreenect/build/lib \
			-L OpenNI2/Bin/x64-Release/OpenNI2/Drivers \
			-L OpenNI2/Bin/x64-Release

VPATH = src \
		obj

KINECT_SRCS = main.cpp \
			  openni_device_listener.cpp \
			  kinect_receiver.cpp \
			  frame_callback.cpp

KINECT_OBJS = $(KINECT_SRCS:.cpp=.o)

kinect: obj/main.o $(KINECT_OBJS:%=obj/%)
	$(CXX) $(LINK_PATH) $^ -o $@ -fPIC -l opencv_core \
									   -l opencv_flann \
									   -l opencv_highgui \
									   -l opencv_imgcodecs \
									   -l opencv_imgproc \
									   -l freenect \
									   -l OpenNI2 \

clean:
	rm obj/*.o
	rm kinect

obj/main.o: main.cpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@

obj/%.o: %.cpp %.hpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@
