CXX = g++
INCDIRS = -I Utility #-I MCU -I Environment
OPT = -O2
#CPPFLAGS = -g -Wall -Wextra -Werror $(INCDIRS) $(OPT)
CPPFLAGS = -g $(INCDIRS) $(OPT)

CPPFILES = main.cpp Quadrotor.cpp Environment.cpp Motor.cpp Coordinate_Frames.cpp Linear_Algebra.cpp
OBJECTS := $(CPPFILES:.cpp=.o)

BINARY = sim

all: $(BINARY)

$(BINARY): $(OBJECTS)
		$(CXX) -o $@ $^

%.o:%.cpp
		$(CXX) $(CPPFLAGS) -c -o $@ $^

%.o:Utility/%.cpp
		$(CXX) $(CPPFLAGS) -c -o $@ $^

# %.o:MCU/%.cpp
# 		$(CXX) $(CPPFLAGS) -c -o $@ $^

# %.o:Environment/%.cpp
# 		$(CXX) $(CPPFLAGS) -c -o $@ $^

clean:
		rm -rf $(BINARY) $(OBJECTS)