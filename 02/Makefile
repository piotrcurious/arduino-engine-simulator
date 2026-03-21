CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra $(shell fltk-config --cxxflags)
LDFLAGS  = $(shell fltk-config --ldflags) -lpthread
SRCS     = engine_sim.cpp main.cpp
OBJS     = $(SRCS:.cpp=.o)
TARGET   = engine_simulator

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

engine_sim.o: engine_sim.cpp engine_sim.h mock_arduino.h
main.o:       main.cpp       engine_sim.h mock_arduino.h TimerOne.h

clean:
	rm -f $(OBJS) $(TARGET)
