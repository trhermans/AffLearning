C++ = g++-4.0

C++-OPT = -O3 -DNDEBUG
C++-DEBUG = -g -O0
C++-WARNS = -Wall
C++-FLAGS = $(C++-WARNS) $(C++-OPT)
#C++-FLAGS = $(C++-WARNS) $(C++-DEBUG)
RM = rm -f

CV_INCLUDE = -I/usr/local/include
INCLUDE = -I./ $(CV_INCLUDE)

# Libraries
LIB_DIRS = -L/usr/local/lib/

CV_LIBS = -lcv -lcvaux -lcxcore -lhighgui

LDLIBS = $(LIB_DIRS) $(CV_LIBS)

# Directory to place all object files
OBJ_DIR = ./build

# Source file listings

BG_SRCS = bg_subtract.cpp \
	  bg_subtract.h

# List of object files
OBJS =  $(OBJ_DIR)/bg_subtract.o

# List of executables
EXECS = bg_test

# Compilation rules
default: $(EXECS)
remake: clean $(EXECS)

bg_test: bg_test.cpp bg_subtract.cpp bg_subtract.h
	$(C++) $(C++-FLAGS) $(INCLUDE) $(LDLIBS) -o $@ bg_test.cpp bg_subtract.cpp bg_subtract.h

bg_test_old: $(OBJ_DIR)/bg_test.cpp bg_subtract.cpp bg_subtact.h
	$(C++) $(C++-FLAGS) $(INCLUDE) $(LDLIBS) $(OBJS) -o $@ $(OBJ_DIR)/bg_test.o

$(OBJ_DIR)/bg_test.o: bg_test.cpp $(OBJS)
	$(C++) $(C++-FLAGS) $(INCLUDE) -c $< -o $@

#
# Objects
#
$(OBJ_DIR)/bg_subtract.o: $(BG_SRCS)
	$(C++) $(C++-FLAGS) $(INCLUDE) -c $< -o $@

.Phony : clean

clean :
	$(RM) $(EXECS) $(OBJ_DIR)/*.o