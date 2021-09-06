CC=gcc
CXX=g++
#DEBUG=-g -ggdb -O0
DEBUG=-Ofast
CFLAGS=-std=c99
CXXFLAGS=-std=c++11
INC=-I/usr/include/eigen3
OBJS=inout.o pathfinder.o
EXE=pathfinder

all : $(EXE)

%.o : %.cpp
	${CXX} ${CXXFLAGS} ${DEBUG} ${INC} -c $<

%.o : %.c
	${CC} $(CFLAGS) ${DEBUG} -c $<

$(EXE) : pathfinder.cpp $(OBJS)
	${CXX} $(CXXFLAGS) ${DEBUG} -o $@ $(OBJS) $(LDFLAGS) -lm -lpng

clean :
	rm -f *.o $(EXE)
