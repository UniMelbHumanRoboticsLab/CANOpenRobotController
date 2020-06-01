# Makefile for Tests
# CXX - defines the compiler. 
# Currently uses the cross-compiler for the BBB (could be modified to run on Windows PC if desired)
CXX := arm-linux-gnueabihf-g++
LD  := arm-linux-gnueabihf-g++

# CXXFLAGS - flags for the compilation 
# -std=c++11 - uses C++11 standard
# -Wno-psabi - does not print the warnings associated with use of the MAP class. 
# -Wwrite-strings - removes warnings re conversion of string constants to char*
CXXFLAGS := -std=c++11  -Wno-psabi  -I eigen 

# LINKFLAGS - flags for linking the objects
# -static used to include the libraries in the executable (originally added for GLIBCXX strings)
LINKFLAGS := -static 

#Define header directories 
MODULES   := core/CANopen/CANopenNode core/CANopen/CANopenNode/stack core/CANopen/CANopenNode/stack/socketCAN \
			 core core/CANopen/CANcomms core/CANopen/objDict core/robot core/robot/joint core/stateMachine core/TrajectoryGenerator \
			 hardware/drives hardware/IO hardware/platforms/example \
			 apps/example apps/example/stateMachine apps/example/stateMachine/states apps/example/TrajectoryGenerator 

# automatically create list of module file paths NOT including executables
SRC_DIR	  := $(addprefix src/,$(MODULES))

# Expand all SRC files from SRC_DIR
SRC_CPP		:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp)) 
SRC_C		:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c))

# Create list of all desired .o files from SRC files in listed MODULES (NOT including Executables)
OBJ_CPP     := $(patsubst src/%.cpp,build/%.o,$(SRC_CPP)) 
OBJ_C       := $(patsubst src/%.c,build/%.o,$(SRC_C)) 

# Objects from the sources, plus the executables
BUILD_DIR := $(addprefix build/,$(MODULES)) build/tests

# List of Test Programs (Executables)
# TESTS	:= testDrives testJoints testOD testRobot testSM testTraj 
TESTS	:= tests/testDrives 

# Test Program Objects and executables
TESTOBJS := $(addsuffix .o, $(addprefix build/tests/,$(TESTS))) 
TESTEXE := $(addprefix build/,$(TESTS))

# For main program
MAIN 	:= build/apps/example/main.o
MAINEXE := build/EXO_APP_2020

# Tell compiler where to find all source files
INCLUDES  := $(addprefix -I,$(SRC_DIR))

# Define a macro which defines a Make Rule for all files in all Source Directories 
define make-goal-cpp
$1: $(subst .o,.cpp,$(subst build,src,$1))
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $$< -o $$@	
endef
$(foreach obj,$(OBJ_CPP),$(eval $(call make-goal-cpp,$(obj))))
$(foreach obj,$(TESTOBJS),$(eval $(call make-goal-cpp,$(obj))))

define make-goal-c
$1: $(subst .o,.c,$(subst build,src,$1))
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $$< -o $$@	
endef
$(foreach obj,$(OBJ_C),$(eval $(call make-goal-c,$(obj))))

# To make the main object
$(MAIN): $(subst .o,.cpp,$(subst build,src,$(MAIN)))
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@	

# To make the main executable
main: $(MAIN) $(OBJ_CPP) $(OBJ_C) 
	$(LD) $(LINKFLAGS) $^ -o $(MAINEXE) -lpthread 

# Define a macro which defines a Make Rule to link tests together
# -lpthread used for to allow pthread_mutex stuff.... (ASK WILL)
define make-tests
$1: build/tests/$1.o $(OBJ_CPP) $(OBJ_C)
	$(LD) $(LINKFLAGS) $$^  -o build/$$@ -lpthread 
endef
$(foreach test,$(TESTS),$(eval $(call make-tests,$(test))))

.PHONY: all checkdirs clean

tests: checkdirs $(TESTS)

checkdirs: $(BUILD_DIR)

exe: checkdirs main

#On Windows - Substitute with command at end of file for UNIX-based systems
# $(BUILD_DIR):
# 	@mkdir $(subst /,\\,$@)

# #
clean:
	@rm -rf $(OBJ_CPP) $(OBJ_C) $(TESTOBJS)$(MAIN) $(MAINEXE) $(BUILD_DIR)

#On UNIX
$(BUILD_DIR):
	@mkdir -p $@ 
