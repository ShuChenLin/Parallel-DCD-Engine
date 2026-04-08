CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude -fopenmp
LDFLAGS  = -fopenmp

GLFW_CFLAGS  = $(shell pkg-config --cflags glfw3 2>/dev/null)
GLFW_LIBS    = $(shell pkg-config --libs glfw3 2>/dev/null)
GL_LIBS      = -lGL

COMMON_SRC = src/morton.cpp src/bvh_seq.cpp src/gjk_seq.cpp src/scene.cpp
COMMON_OBJ = $(COMMON_SRC:.cpp=.o)

all: dcd_seq dcd_viz dcd_test

dcd_seq: src/main.o $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

dcd_viz: src/renderer.o $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(GLFW_LIBS) $(GL_LIBS) $(LDFLAGS)

src/renderer.o: src/renderer.cpp
	$(CXX) $(CXXFLAGS) $(GLFW_CFLAGS) -c -o $@ $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

dcd_test: tests/test_all.o $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test: dcd_test
	./dcd_test

clean:
	rm -f src/*.o tests/*.o dcd_seq dcd_viz dcd_test

.PHONY: all clean test
