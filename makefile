UNAME_S := $(shell uname -s)
C_INCLUDE_PATH=/opt/X11/include
export C_INCLUDE_PATH
ifeq ($(UNAME_S),Linux)
CC = g++
CPP_FLAGS = -Wall -O4 -w -fopenmp -std=c++11 -pthread
endif
ifeq ($(UNAME_S),Darwin)
CC = clang++
CPP_FLAGS =  -Xpreprocessor -fopenmp -lomp -Wall -O4 -w -std=c++11 -stdlib=libc++ -I/opt/X11/include -L/opt/X11/lib -lm -lpthread -lX11
endif
FILES = RayTracer.cpp materialObject.cpp displayObject.cpp lightObject.cpp cameraObject.cpp
EXE = raytracer

build: $(FILES)
	$(CC) $(CPP_FLAGS) -o $(EXE) $(FILES)

clean:
	rm -f *.o *~ $(EXE)
