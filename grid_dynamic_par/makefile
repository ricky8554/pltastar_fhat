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
cpp_files1 = world.cpp camera.cpp obstacle.cpp LSS_LRTA.cpp PLTASTAR.cpp LSS_LRTA_FHAT.cpp PLTASTAR_FHAT.cpp PLTASTAR_FHAT_MOD.cpp PLTASTAR_MOD.cpp
cpp_files2 = control.cpp obstacle.cpp LSS_LRTA.cpp PLTASTAR.cpp PLTASTAR_MOD.cpp LSS_LRTA_FHAT.cpp PLTASTAR_FHAT.cpp PLTASTAR_FHAT_MOD.cpp
#
c_files = deps/glad.c

objects1 = $(cpp_files1:.cpp=.o) $(c_files:.c=.o)
objects2 = $(cpp_files2:.cpp=.o) $(c_files:.c=.o)

all: $(TARGET1) $(TARGET2)

test: hashtest

$(TARGET1): $(objects1) 
	$(CXX) -o $@ $^ $(LDFLAGS)

$(TARGET2): $(objects2) 
	$(CXX) -o $@ $^

hashtest: hashtest.cpp HashSet.cpp HashSet.h
	$(CXX)  -std=c++11 hashtest.cpp -g -o hashtest




.PHONY : clean
clean :
	-rm $(TARGET1) $(objects1)
	-rm $(TARGET2)


