CC = gcc
CXX = g++

INCLUDES = -I$(glad_inc) -I/usr/local/include
LIBRARIES = 

source_dir = .

glad_inc = $(source_dir)/deps

CFLAGS = -Wall -ggdb -O3 $(INCLUDES)
CXXFLAGS = -Wall -ggdb -O3 $(INCLUDES) -std=c++11
CXXFLAGS1 = -Wall -O2 $(INCLUDES) -std=c++11
LDFLAGS = $(LIBRARIES) -L/usr/local/lib -lglfw3 -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo

TARGET1 = world

TARGET2 = control

#skeloton.cpp 
cpp_files1 = executive/world.cpp utility/camera.cpp executive/executive.cpp planner/LSS_LRTA.cpp planner/PLTASTAR.cpp planner/LSS_LRTA_FHAT.cpp planner/PLTASTAR_FHAT.cpp planner/PLTASTAR_FHAT_MOD.cpp planner/PLTASTAR_MOD.cpp
cpp_files2 = executive/executive.cpp executive/control.cpp planner/LSS_LRTA.cpp planner/PLTASTAR.cpp planner/PLTASTAR_MOD.cpp planner/LSS_LRTA_FHAT.cpp planner/PLTASTAR_FHAT.cpp planner/PLTASTAR_FHAT_MOD.cpp
#
c_files = deps/glad.c

objects1 = $(cpp_files1:.cpp=.o) $(c_files:.c=.o)
objects2 = $(cpp_files2:.cpp=.o) $(c_files:.c=.o)

all: $(TARGET1) $(TARGET2)

test: hashtest

$(TARGET1): $(objects1) 
	$(CXX) -o $@ $^ $(LDFLAGS)

$(TARGET2): $(objects2) 
	$(CXX) -o $@ $^ $(LDFLAGS)

hashtest: hashtest.cpp HashSet.cpp HashSet.h
	$(CXX)  -std=c++11 hashtest.cpp -g -o hashtest




.PHONY : clean
clean :
	-rm $(TARGET1) $(objects1)
	-rm $(TARGET2)


