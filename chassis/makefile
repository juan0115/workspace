CXX ?= g++

CXXFLAGS += -c -Wall $(shell pkg-config --cflags opencv)
LDFLAGS += $(shell pkg-config --libs --static opencv)

test3 : main.o dprocess.o serial.o 
	$(CXX) -o chassis main.o dprocess.o serial.o -lpthread $(LDFLAGS)

main.o : main.cpp dprocess.h
	$(CXX) -c main.cpp $(CXXFLAGS)

serial.o : serial.cpp serial.h afx.h
	$(CXX) -c serial.cpp $(CXXFLAGS)

dprocess.o : dprocess.cpp dprocess.h  serial.h
	$(CXX) -c dprocess.cpp $(CXXFLAGS)

clean :
	rm -f chassis main.o dprocess.o serial.o
