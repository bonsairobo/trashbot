CXX = g++
CXX_FLAGS = -O3 -Wall -pedantic -std=c++11 -c
INCLUDE_PATH = -I libfreenect/include \
			   -I OpenNI2/Include \
			   -I ./pcl/build/include \
			   -I .. \
			   -I /usr/local/include/pcl-1.8 \

LINK_PATH = -L opencv/build/lib \
			-L libfreenect/build/lib \
			-L OpenNI2/Bin/x64-Debug/OpenNI2/Drivers \
			-L OpenNI2/Bin/x64-Debug \
			-L ../common \
			-L /usr/local/lib

VPATH = src \
		obj

KINECT_SRCS = main.cpp \
			  img_proc.cpp \
			  common.cpp \
			  occupancy_grid.cpp \
			  trash_search.cpp

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
									   -l common \
									   -l pcl_common \
									   -l pcl_features \
									   -l pcl_search \
									   -l pcl_segmentation \
									   -l pcl_filters

file_analysis: obj/file_analysis.o obj/img_proc.o obj/common.o obj/occupancy_grid.o
	$(CXX) -g $(LINK_PATH) $^ -o $@ -fPIC -l opencv_core \
									   -l opencv_highgui \
									   -l opencv_imgcodecs \
									   -l opencv_imgproc \
									   -l OpenNI2 \
									   -l pcl_common \
									   -l pcl_features \
									   -l pcl_search \
									   -l pcl_segmentation \
									   -l pcl_filters

clean:
	rm obj/*.o
	rm kinect
	rm file_analysis

obj/main.o: main.cpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@

obj/file_analysis.o: file_analysis.cpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@

obj/%.o: %.cpp %.hpp
	$(CXX) $(INCLUDE_PATH) $(CXX_FLAGS) $< -o $@
