CXX = g++
CXXFLAGS = -std=c++17 -I. -Ivibe_test/mock

SRCS = vibe_test/engine.cpp vibe_test/test_main.cpp vibe_test/mock/Arduino.cpp vibe_test/mock/MockSystem.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = engine_test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)
