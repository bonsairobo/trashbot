CXX = g++
CXX_FLAGS = -fPIC -Wall -pedantic -std=c++11 -c
LD_FLAGS = -shared

COMMON_SRCS = socket_types.cpp
COMMON_OBJS = $(COMMON_SRCS:.cpp=.o)

libcommon.so: $(COMMON_OBJS:%=obj/%)
	$(CXX) ${LD_FLAGS} -o $@ $^

clean:
	rm obj/*.o
	rm libcommon.so

obj/%.o: %.cpp %.hpp
	$(CXX) $(CXX_FLAGS) $< -o $@
