CXX = g++
CC = gcc
CXXFLAGS = -g -Wall -O0
CFLAGS = -g -Wall -O0
CPPFLAGS = -MMD -MP -I Utility -I Sensors -I PIC32

BUILD_DIR = Build

CXXSOURCES = $(wildcard *.cpp) $(wildcard Utility/*.cpp) $(wildcard Sensors/*.cpp) 
CXXOBJECTS = $(CXXSOURCES:%.cpp=$(BUILD_DIR)/%.o)
CXXDEPENDS = $(CXXOBJECTS:.o=.d)

CCSOURCES = $(wildcard PIC32/*.c)
CCOBJECTS = $(CCSOURCES:%.c=$(BUILD_DIR)/%.o)
CCDEPENDS = $(CCSOURCES:.o=.d)

$(BUILD_DIR)/sim.exe: $(CXXOBJECTS) $(CCOBJECTS) 
	$(CXX) $^ -o $@

$(BUILD_DIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CCFLAGS) -c $< -o $@

-include $(DEPENDS)


.PHONY: clean

clean:
	rm -rf sim.exe $(CXXOBJECTS) $(CCOBJECTS)

.PHONY: clean_dep

clean_dep:
	rm -rf $(CXXDEPENDS) $(CCDEPENDS)

.PHONY: pristine

pristine:
	rm -rf sim.exe $(CXXDEPENDS) $(CXXOBJECTS) $(CCDEPENDS) $(CCDEPENDS)