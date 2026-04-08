CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude

# GLFW / OpenGL (macOS)
GLFW_CFLAGS  = $(shell pkg-config --cflags glfw3 2>/dev/null || echo "-I/opt/homebrew/include")
GLFW_LIBS    = $(shell pkg-config --libs glfw3 2>/dev/null || echo "-L/opt/homebrew/lib -lglfw")
GL_LIBS      = -framework OpenGL

COMMON_SRC = src/morton.cpp src/bvh_seq.cpp src/gjk_seq.cpp src/scene.cpp
COMMON_OBJ = $(COMMON_SRC:.cpp=.o)

all: dcd_seq dcd_viz dcd_test

dcd_seq: src/main.o $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

dcd_viz: src/renderer.o $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(GLFW_LIBS) $(GL_LIBS)

src/renderer.o: src/renderer.cpp
	$(CXX) $(CXXFLAGS) $(GLFW_CFLAGS) -c -o $@ $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

dcd_test: tests/test_all.o $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

test: dcd_test
	./dcd_test

clean:
	rm -f src/*.o tests/*.o dcd_seq dcd_viz dcd_test

.PHONY: all clean test
