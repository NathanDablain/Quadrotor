CXX = g++
CC = gcc
CXXFLAGS = -Wall -O2
CXXDBGFLAGS = -g -Wall -O0
CFLAGS = -g -Wall -O0
CPPFLAGS = -MMD -MP -I Utility -I Sensors -I PIC32

BUILD_DIR_DBG = Build_Debug
BUILD_DIR_REL = Build_Release

CXXSOURCES = $(wildcard *.cpp) $(wildcard Utility/*.cpp) $(wildcard Sensors/*.cpp) $(wildcard PIC32/*.cpp)
CXXDBGOBJECTS = $(CXXSOURCES:%.cpp=$(BUILD_DIR_DBG)/%.o)
CXXOBJECTS = $(CXXSOURCES:%.cpp=$(BUILD_DIR_REL)/%.o)
CXXDBGDEPENDS = $(CXXDBGOBJECTS:.o=.d)
CXXDEPENDS = $(CXXOBJECTS:.o=.d)

CCSOURCES = $(wildcard PIC32/*.c)
CCOBJECTS = $(CCSOURCES:%.c=$(BUILD_DIR_REL)/%.o)
CCDBGOBJECTS = $(CCSOURCES:%.c=$(BUILD_DIR_DBG)/%.o)
CCDEPENDS = $(CCOBJECTS:.o=.d)
CCDBGDEPENDS = $(CCDBGOBJECTS:.o=.d)

default: all

all: $(BUILD_DIR_REL)/sim.exe $(BUILD_DIR_DBG)/sim_dbg.exe

release: $(BUILD_DIR_REL)/sim.exe

debug: $(BUILD_DIR_DBG)/sim_dbg.exe

$(BUILD_DIR_REL)/sim.exe: $(CXXOBJECTS) $(CCOBJECTS) 
	$(CXX) $^ -o $@

$(BUILD_DIR_DBG)/sim_dbg.exe: $(CXXDBGOBJECTS) $(CCDBGOBJECTS) 
	$(CXX) $^ -o $@

$(BUILD_DIR_REL)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR_REL)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CCFLAGS) -c $< -o $@

$(BUILD_DIR_DBG)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXDBGFLAGS) -c $< -o $@

$(BUILD_DIR_DBG)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CCDBGFLAGS) -c $< -o $@

-include $(CXXDEPENDS) $(CCDEPENGS) $(CXXDBGDEPENDS) $(CCDBGDEPENDS)


.PHONY: clean

clean:
	rm -rf $(BUILD_DIR_REL)/sim.exe $(BUILD_DIR_DBG)/sim_dbg.exe $(CXXOBJECTS) $(CCOBJECTS) $(CXXDBGOBJECTS) $(CCDBGOBJECTS)

.PHONY: clean_dep

clean_dep:
	rm -rf $(CXXDEPENDS) $(CCDEPENDS) $(CXXDBGDEPENDS) $(CCDBGDEPENDS)

.PHONY: pristine

pristine:
	rm -rf $(BUILD_DIR_REL)/sim.exe $(BUILD_DIR_DBG)/sim_debg.exe $(CXXDEPENDS) $(CXXOBJECTS) $(CCDEPENDS) $(CCDEPENDS) $(CXXDBGOBJECTS) $(CCDBGOBJECTS) $(CXXDBGDEPENDS) $(CCDBGDEPENDS)