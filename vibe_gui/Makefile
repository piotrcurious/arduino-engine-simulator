CXX = g++
CXXFLAGS = -std=c++17 -Wall -I. `fltk-config --cxxflags`
LDFLAGS = `fltk-config --ldflags`

SRCS = engine.cpp main.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = vibe_sim_gui

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)
