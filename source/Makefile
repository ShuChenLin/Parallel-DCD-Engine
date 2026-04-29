UNAME_S := $(shell uname -s)

BASE_CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude

ifeq ($(UNAME_S),Darwin)
CXX        = clang++
OMP_PREFIX ?= $(shell brew --prefix libomp 2>/dev/null)
OMP_CXXFLAGS = -Xpreprocessor -fopenmp -I$(OMP_PREFIX)/include
OMP_LDFLAGS  = -L$(OMP_PREFIX)/lib -lomp
GL_LIBS      = -framework OpenGL
else
CXX        = g++
OMP_CXXFLAGS = -fopenmp
OMP_LDFLAGS  = -fopenmp
GL_LIBS      = -lGL
endif

CXXFLAGS = $(BASE_CXXFLAGS) $(OMP_CXXFLAGS)
LDFLAGS  = $(OMP_LDFLAGS)

GLFW_CFLAGS  = $(shell pkg-config --cflags glfw3 2>/dev/null)
GLFW_LIBS    = $(shell pkg-config --libs glfw3 2>/dev/null)

<<<<<<< Updated upstream
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
=======
# Shared sources (init, bruteforce, GJK)
COMMON_SRC = src/scene_common.cpp src/seq/gjk_seq.cpp
>>>>>>> Stashed changes

# Sequential pipeline
SEQ_SRC = src/seq/scene_seq.cpp src/seq/bvh_seq.cpp src/seq/morton_seq.cpp

<<<<<<< Updated upstream
# Sequential CLI
dcd_seq: src/seq/main.o $(SHARED_OBJ) $(SEQ_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Visualizer (sequential)
dcd_viz: src/renderer.o $(SHARED_OBJ) $(SEQ_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(GLFW_LIBS) $(GL_LIBS)
=======
# OpenMP pipeline
OMP_SRC = src/omp/scene_omp.cpp src/omp/bvh_omp.cpp src/omp/morton_omp.cpp

ALL_SRC = $(COMMON_SRC) $(SEQ_SRC) $(OMP_SRC)
ALL_OBJ = $(ALL_SRC:.cpp=.o)

all: dcd

dcd: src/main.o $(ALL_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

dcd_viz: src/renderer.o $(ALL_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(GLFW_LIBS) $(GL_LIBS) $(LDFLAGS)
>>>>>>> Stashed changes

src/renderer.o: src/renderer.cpp
	$(CXX) $(CXXFLAGS) $(GLFW_CFLAGS) -c -o $@ $<

<<<<<<< Updated upstream
# OpenMP CLI (compile OMP sources with -fopenmp, link seq/gjk_seq for GJK)
dcd_omp: src/omp/main_omp.o $(SHARED_OBJ) $(OMP_OBJ) src/seq/gjk_seq.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

src/omp/%.o: src/omp/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Tests (sequential)
dcd_test: tests/test_all.o $(SHARED_OBJ) $(SEQ_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

=======
dcd_test: tests/test_all.o $(ALL_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

>>>>>>> Stashed changes
test: dcd_test
	./dcd_test

# CUDA
NVCC     = nvcc
# Use GCC 11 as host compiler (CUDA 11.7 is incompatible with GCC 13+ headers)
NVHOST   = $(shell which g++-11 2>/dev/null || echo g++)
NVFLAGS  = -std=c++17 -O2 -Iinclude --extended-lambda -ccbin $(NVHOST)

CUDA_SRC = src/cuda/scene_cuda.cu src/cuda/main_cuda.cu
CUDA_OBJ = $(CUDA_SRC:.cu=.o)

dcd_cuda: $(CUDA_OBJ) $(SHARED_OBJ) src/seq/gjk_seq.o
	$(NVCC) $(NVFLAGS) -o $@ $^

src/cuda/%.o: src/cuda/%.cu
	$(NVCC) $(NVFLAGS) -c -o $@ $<

clean:
<<<<<<< Updated upstream
	rm -f src/*.o src/seq/*.o src/omp/*.o src/cuda/*.o tests/*.o dcd_seq dcd_viz dcd_test dcd_omp dcd_cuda
=======
	rm -f src/*.o src/seq/*.o src/omp/*.o tests/*.o dcd dcd_viz dcd_test
>>>>>>> Stashed changes

.PHONY: all clean test
