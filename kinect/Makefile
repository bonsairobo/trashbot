CXX = g++
CXX_FLAGS = -Wall -pedantic -std=c++11 -c
INCLUDE_PATH = -I libfreenect/include -I OpenNI2/Include
LINK_PATH = -L opencv/build/lib \
			-L libfreenect/build/lib \
			-L OpenNI2/Bin/x64-Release/OpenNI2/Drivers \
			-L OpenNI2/Bin/x64-Release \
			-L ../common

VPATH = src \
		obj

KINECT_SRCS = main.cpp \
			  localization_model.cpp \
			  grasping_model.cpp \
			  img_proc.cpp \
			  common.cpp

KINECT_OBJS = $(KINECT_SRCS:.cpp=.o)

all: kinect file_analysis

kinect: obj/main.o $(KINECT_OBJS:%=obj/%)
	$(CXX) $(LINK_PATH) $^ -o $@ -fPIC -l opencv_core \
									   -l opencv_flann \
									   -l opencv_highgui \
									   -l opencv_imgcodecs \
									   -l opencv_imgproc \
									   -l opencv_videoio \
									   -l freenect \
									   -l OpenNI2 \
									   -l common

file_analysis: obj/file_analysis.o obj/img_proc.o obj/common.o
	$(CXX) $(LINK_PATH) $^ -o $@ -fPIC -l opencv_core \
									   -l opencv_highgui \
									   -l opencv_imgcodecs \
									   -l opencv_imgproc \
									   -l OpenNI2

clean:
	rm kinect
	rm file_analysis
	rm obj/*.o

obj/main.o: main.cpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@

obj/file_analysis.o: file_analysis.cpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@

obj/%.o: %.cpp %.hpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@
