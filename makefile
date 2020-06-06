UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
CC = g++
CPP_FLAGS = -Wall -O4 -w -fopenmp -std=c++11 -pthread
endif
ifeq ($(UNAME_S),Darwin)
CC = clang++
CPP_FLAGS =  -Xpreprocessor -fopenmp -lomp -Wall -O4 -w -std=c++11 -stdlib=libc++ -I./X11/include -L./X11/lib -lm -lpthread -lX11 -Dcimg_use_png -lpng -lz -g
endif
FILES = RayTracer.cpp materialObject.cpp displayObject.cpp lightObject.cpp cameraObject.cpp normalObject.cpp sphereObject.cpp sharedVertexObject.cpp modelObject.cpp rayObject.cpp
EXE = raytracer

build: $(FILES)
	$(CC) $(CPP_FLAGS) -o $(EXE) $(FILES)

clean:
	rm -f *.o *~ $(EXE)
