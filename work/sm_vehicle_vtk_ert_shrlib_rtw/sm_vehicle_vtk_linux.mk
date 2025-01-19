###########################################################################
## Makefile generated for component 'sm_vehicle_vtk'. 
## 
## Makefile     : sm_vehicle_vtk.mk
## Generated on : Mon Aug 19 22:55:53 2024
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/sm_vehicle_vtk.so
## Product type : shared library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

PRODUCT_NAME              = sm_vehicle_vtk
MAKEFILE                  = sm_vehicle_vtk.mk
MATLAB_ROOT               = /usr/local/MATLAB/R2024a
MATLAB_BIN                = $(MATLAB_ROOT)/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = /home/user/Projects/Vehicle-Dynamics-14DOF-Simscape-Multibody/work
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = ISO_C
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ..
COMPILER_COMMAND_FILE     = sm_vehicle_vtk_comp.rsp
CMD_FILE                  = sm_vehicle_vtk.rsp
DEF_FILE                  = $(PRODUCT_NAME).def
C_STANDARD_OPTS           = -fwrapv
CPP_STANDARD_OPTS         = -fwrapv
LIBSSC_SLI_OBJS           = 
LIBSM_SSCI_OBJS           = 
LIBSSC_CORE_OBJS          = 
LIBSSC_ST_OBJS            = 
LIBMC_OBJS                = 
LIBSM_OBJS                = 
LIBPM_MATH_OBJS           = 
LIBPM_OBJS                = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

WARN_FLAGS            = -Wall -Wextra -Wwrite-strings -Winline -Wstrict-prototypes -Wpointer-arith -Wcast-align -Wno-stringop-overflow
WARN_FLAGS_MAX        = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS        = -Wall -Wextra -Wwrite-strings -Winline -Wpointer-arith -Wcast-align -Wno-stringop-overflow
CPP_WARN_FLAGS_MAX    = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow
MW_EXTERNLIB_DIR      = $(MATLAB_ROOT)/extern/lib/glnxa64
SHELL                 = /bin/bash

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm -lpthread

###########################################################################
## BUILD TOOL COMMANDS
###########################################################################

# C Compiler: GNU C Compiler
CC = gcc

# Linker: GNU Linker
LD = g++

# C++ Compiler: GNU C++ Compiler
CPP = g++

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX = $(MATLAB_ARCH_BIN)/mex

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GNU Make Utility
MAKE = make

###########################################################################
## Directives/Utilities
###########################################################################

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
ARDEBUG             = 
STATICLIB_OUTPUT_FLAG = 
MEX_DEBUG           = -g
RM                  = rm -f
ECHO                = echo
MV                  = mv
RUN                 =

###########################################################################
## "Faster Runs" Build Configuration
###########################################################################

ARFLAGS              = ruvs
CFLAGS               = -c $(C_STANDARD_OPTS) -O3 -m64
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -O3 -m64
CPP_LDFLAGS          = -static -m64
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,--no-undefined \
                         -Wl,-soname,$(notdir $(basename $(PRODUCT))).so
LDFLAGS              = -static -m64
SHAREDLIB_LDFLAGS    = -shared -Wl,--no-undefined \
                       -Wl,-soname,$(notdir $(basename $(PRODUCT))).so

###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/sm_vehicle_vtk.so
PRODUCT_TYPE = "shared library"
BUILD_TYPE = "Shared Library Target"

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/sm_vehicle_vtk_ert_shrlib_rtw/sm_vehicle_vtk_836bb176_1.c \
       $(START_DIR)/sm_vehicle_vtk_ert_shrlib_rtw/sm_vehicle_vtk_836bb176_1_create.c \
       $(START_DIR)/sm_vehicle_vtk_ert_shrlib_rtw/sm_vehicle_vtk_836bb176_1_setParameters.c \
       # (additional sources omitted for brevity)

MAIN_SRC = $(START_DIR)/sm_vehicle_vtk_ert_shrlib_rtw/ert_main.c
ALL_SRCS = $(SRCS) $(MAIN_SRC)

###########################################################################
## OBJECTS
###########################################################################

OBJS = sm_vehicle_vtk_836bb176_1.o sm_vehicle_vtk_836bb176_1_create.o sm_vehicle_vtk_836bb176_1_setParameters.o # (additional objects omitted)

MAIN_OBJ = ert_main.o
ALL_OBJS = $(OBJS) $(MAIN_OBJ)

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(MATLAB_ROOT)/extern/physmod/glnxa64/ssc_sli/lib/ssc_sli_glnxa64.a \
       $(MATLAB_ROOT)/extern/physmod/glnxa64/sm_ssci/lib/sm_ssci_glnxa64.a # (additional libraries omitted)

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

CFLAGS_TFL = -msse2 -fno-predictive-commoning
CPPFLAGS_TFL = -msse2 -fno-predictive-commoning

CFLAGS += $(CFLAGS_TFL) $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)
CPPFLAGS += $(CPPFLAGS_TFL) $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute

all : build
	@echo "### Successfully generated all binary outputs."

build : prebuild $(PRODUCT)

prebuild : 

$(PRODUCT) : $(OBJS) $(LIBS)
	@echo "### Creating shared library $(PRODUCT) ..."
	$(LD) $(SHAREDLIB_LDFLAGS) -o $(PRODUCT) @$(CMD_FILE) -Wl,--start-group $(LIBS) -Wl,--end-group $(TOOLCHAIN_LIBS)
	@echo "### Created: $(PRODUCT)"

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"

%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"

###########################################################################
## CLEAN TARGET
###########################################################################

clean:
	$(RM) $(ALL_OBJS) $(PRODUCT)
