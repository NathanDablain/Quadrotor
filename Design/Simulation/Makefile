CXX = g++
CXXFLAGS = -g -Wall -MMD -O0 -I Utility

SOURCES = $(wildcard *.cpp) $(wildcard Utility/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
DEPENDS = $(SOURCES:.cpp=.d)
-include $(DEPENDS)

sim.exe: $(OBJECTS)
	$(CXX) $(CSSFLAGS) -o $@ $^

.PHONY: clean

clean:
	rm -f sim.exe $(OBJECTS) 

.PHONY: clean_dep

clean_dep:
	rm -f $(DEPENDS)