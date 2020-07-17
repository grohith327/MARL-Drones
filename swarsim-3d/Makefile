# Sources from ANN: https://www.cs.umd.edu/~mount/ANN/.
# These source cannot be modified according to license, but can be distributed.
ANN_SOURCES = ann/ANN.cpp ann/brute.cpp ann/kd_tree.cpp ann/kd_util.cpp ann/kd_split.cpp \
              ann/kd_dump.cpp ann/kd_search.cpp ann/kd_pr_search.cpp ann/kd_fix_rad_search.cpp \
              ann/bd_tree.cpp ann/bd_search.cpp ann/bd_pr_search.cpp ann/bd_fix_rad_search.cpp \
              ann/perf.cpp

# Handling flags.
GFLAGS_SOURCES = gflags/gflags.cc gflags/gflags_completions.cc gflags/gflags_reporting.cc

# Simulator sources.
SIM_SOURCES = display/window.cc display/texture_manager.cc display/model_3ds.cc \
              main/swarmsim.cc util/utils.cc \
              core/supervisor.cc core/robot.cc core/unicycle.cc core/bicycle.cc core/quadrotor.cc \
              plugin/default_supervisor.cc plugin/default_unicycle.cc plugin/default_quadrotor.cc \
              plugin/python_supervisor.cc plugin/python_unicycle.cc plugin/python_bicycle.cc plugin/python_quadrotor.cc

# User defined code.
USER_SOURCES = $(wildcard user_plugin/*.cc)

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

HAS_CLANG := $(shell command -v clang++ 2> /dev/null)
ifdef HAS_CLANG
  CC = g++ -std=c++11
#   CC = clang++ -std=c++11 -stdlib=libc++
else
  CC = g++ -std=c++11
endif

CFLAGS = -O3 -Wall -I. -Wno-unused-function -fno-strict-aliasing -Wno-write-strings -DUSE_GL -ffast-math -funroll-loops -Wno-unused-const-variable -Wno-deprecated-register
CFLAGS += `python-config --includes`
ifeq ($(OSTYPE), darwin)
  CFLAGS += -DMAC -I/sw/include -I/opt/local/include -Wno-deprecated-declarations
else
  CFLAGS += -Wno-unused-result -Wno-sign-compare -Wno-conversion-null # Apparently not recognized by cc1plus of MACOS
endif

LIBS = -lm -lpthread
ifneq ($(OSTYPE), darwin)
  LIBS += -lGL -lGLU -lglut -lfltk -lfltk_gl
else
  LIBS += -framework GLUT -framework OpenGL -framework Carbon -lfltk -lfltk_gl
endif

OBJECTS = $(USER_SOURCES:.cc=.o) $(SIM_SOURCES:.cc=.o) $(ANN_SOURCES:.cpp=.o) $(GFLAGS_SOURCES:.cc=.o)
LDFLAGS = $(LIBS) `python-config --ldflags`

all: swarmsim

swarmsim:  $(OBJECTS)
	$(CC) $(OBJECTS) $(LIBRARIES) -o $@ $(LDFLAGS)

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

.cc.o:
	$(CC) $(CFLAGS) -c $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJECTS) swarmsim
