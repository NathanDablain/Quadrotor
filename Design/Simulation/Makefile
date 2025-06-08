CXX = g++
CXXFLAGS = -g -Wall -MMD -O0 -I Utility

SOURCES = $(wildcard *.cpp) $(wildcard Utility/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
DEPENDS = $(SOURCES:.cpp=.d)

sim.exe: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

-include $(DEPENDS)

.PHONY: clean

clean:
	rm -rf sim.exe $(OBJECTS) 

.PHONY: clean_dep

clean_dep:
	rm -rf $(DEPENDS)

.PHONY: pristine

pristine:
	rm -rf sim.exe $(DEPENDS) $(OBJECTS)