CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude

# GLFW / OpenGL (macOS)
GLFW_CFLAGS  = $(shell pkg-config --cflags glfw3 2>/dev/null || echo "-I/opt/homebrew/include")
GLFW_LIBS    = $(shell pkg-config --libs glfw3 2>/dev/null || echo "-L/opt/homebrew/lib -lglfw")
GL_LIBS      = -framework OpenGL

# Shared sources (morton utilities, scene init/bruteforce)
SHARED_SRC = src/morton.cpp src/scene_common.cpp

# Sequential sources
SEQ_SRC = src/seq/bvh_seq.cpp src/seq/gjk_seq.cpp src/seq/morton_seq.cpp src/seq/scene_seq.cpp

# OpenMP sources
OMP_SRC = src/omp/bvh_omp.cpp src/omp/morton_omp.cpp src/omp/scene_omp.cpp

# Object files
SHARED_OBJ = $(SHARED_SRC:.cpp=.o)
SEQ_OBJ    = $(SEQ_SRC:.cpp=.o)
OMP_OBJ    = $(OMP_SRC:.cpp=.o)

all: dcd_seq dcd_viz dcd_test

# Sequential CLI
dcd_seq: src/main.o $(SHARED_OBJ) $(SEQ_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Visualizer (sequential)
dcd_viz: src/renderer.o $(SHARED_OBJ) $(SEQ_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(GLFW_LIBS) $(GL_LIBS)

src/renderer.o: src/renderer.cpp
	$(CXX) $(CXXFLAGS) $(GLFW_CFLAGS) -c -o $@ $<

# OpenMP CLI (compile OMP sources with -fopenmp, link seq/gjk_seq for GJK)
dcd_omp: src/main_omp.o $(SHARED_OBJ) $(OMP_OBJ) src/seq/gjk_seq.o
	$(CXX) $(CXXFLAGS) -fopenmp -o $@ $^

src/omp/%.o: src/omp/%.cpp
	$(CXX) $(CXXFLAGS) -fopenmp -c -o $@ $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Tests (sequential)
dcd_test: tests/test_all.o $(SHARED_OBJ) $(SEQ_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

test: dcd_test
	./dcd_test

clean:
	rm -f src/*.o src/seq/*.o src/omp/*.o tests/*.o dcd_seq dcd_viz dcd_test dcd_omp

.PHONY: all clean test
