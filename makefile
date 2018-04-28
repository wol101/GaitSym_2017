# Note use "make -j 8" to use 8 threads
# get the system and architecture
ifeq ($(OS),Windows_NT)
	SYSTEM := WIN32
	ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
		ARCH := AMD64
	endif
	ifeq ($(PROCESSOR_ARCHITEW6432),AMD64)
		ARCH := AMD64
	endif
	ifeq ($(PROCESSOR_ARCHITECTURE),x86)
		ARCH := IA32
	endif
	UNAME_S := $(shell uname -s)
	ifneq (,$(findstring CYGWIN,$(UNAME_S)))
		SYSTEM := CYGWIN
	endif
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Linux)
	SYSTEM := LINUX
	endif
	ifeq ($(UNAME_S),Darwin)
	SYSTEM := OSX
	endif
	UNAME_P := $(shell uname -p)
	ifeq ($(UNAME_P),x86_64)
	ARCH := AMD64
	endif
	ifneq ($(filter %86,$(UNAME_P)),)
	ARCH := IA32
	endif
	ifneq ($(filter arm%,$(UNAME_P)),)
	ARCH := ARM
	endif
	ifneq ($(filter ppc%,$(UNAME_P)),)
	ARCH := PPC
	endif
endif
HOST := $(shell hostname)
cc_AVAIL := $(shell command -v cc 2> /dev/null)
gcc_AVAIL := $(shell command -v gcc 2> /dev/null)
CC_AVAIL := $(shell command -v CC 2> /dev/null)
gpp_AVAIL := $(shell command -v g++ 2> /dev/null)

ifeq ($(SYSTEM),OSX)
	CXXFLAGS = -Wall -fexceptions $(OPT_FLAGS)  \
	-DdIDEDOUBLE -DTRIMESH_ENABLED -DTRIMESH_OPCODE -DCCD_IDEDOUBLE -DdLIBCCD_ENABLED -DdTHREADING_INTF_DISABLED \
	-DRAPIDXML_NO_EXCEPTIONS -DBYTE_ORDER=LITTLE_ENDIAN -DUSE_UNIX_ERRORS -DHAVE_ALLOCA_H
	LDFLAGS = 
	LIBS = -lpthread -lm -framework CoreServices
	INC_DIRS = -Irapidxml-1.13 -Iexprtk -Iode-0.15/ode/src -Iode-0.15/libccd/src -Iode-0.15/OPCODE -Iode-0.15/include -Iann_1.1.2/include
endif

ifeq ($(SYSTEM),LINUX)
	#CXXFLAGS = -static -ffast-math -O3  -DEXPERIMENTAL
	CXXFLAGS = -static -O3 -ffast-math -DEXPERIMENTAL -std=c++11 \
	-DdIDEDOUBLE -DTRIMESH_ENABLED -DTRIMESH_OPCODE -DCCD_IDEDOUBLE -DdLIBCCD_ENABLED -DdTHREADING_INTF_DISABLED \
	-DRAPIDXML_NO_EXCEPTIONS -DBYTE_ORDER=LITTLE_ENDIAN -DHAVE_MALLOC_H -DUSE_UNIX_ERRORS
	LDFLAGS  = -static 
ifdef gcc_AVAIL
	CC       = gcc
endif
ifdef gpp_AVAIL
	CXX      = g++
endif
ifdef cc_AVAIL
	CC       = cc
endif
ifdef CC_AVAIL
	CXX      = CC
endif
	LIBS = -lpthread -lm
	INC_DIRS = -Irapidxml-1.13 -Iexprtk -Iode-0.15/ode/src -Iode-0.15/libccd/src -Iode-0.15/OPCODE -Iode-0.15/include -Iann_1.1.2/include
endif

ifeq ($(SYSTEM),CYGWIN)
	# cygwin64 gcc version
	CXXFLAGS = -O3 -g -ffast-math -Wa,-mbig-obj -Dexprtk_disable_enhanced_features -std=c++11 \
	-DdIDEDOUBLE -DTRIMESH_ENABLED -DTRIMESH_OPCODE -DCCD_IDEDOUBLE -DdLIBCCD_ENABLED -DdTHREADING_INTF_DISABLED \
	-DRAPIDXML_NO_EXCEPTIONS -DBYTE_ORDER=LITTLE_ENDIAN -DHAVE_MALLOC_H -DUSE_UNIX_ERRORS -DHAVE_ALLOCA_H \
	-DALLOCA_H_NEEDED -DNEED_BCOPY -DSTRINGS_H_NEEDED
	LDFLAGS  =
	CXX      = g++
	CC       = gcc
	LIBS = -lpthread -lm
	INC_DIRS = -Irapidxml-1.13 -Iexprtk -Iode-0.15/ode/src -Iode-0.15/libccd/src -Iode-0.15/OPCODE -Iode-0.15/include -Iann_1.1.2/include
endif

ifeq ($(HOST),asl-rocks-cluster.manchester.ac.uk)
	#CXXFLAGS = -static -ffast-math -O3  -DEXPERIMENTAL
	CXXFLAGS = -static -O3 -ffast-math -DEXPERIMENTAL \
	-DdIDEDOUBLE -DTRIMESH_ENABLED -DTRIMESH_OPCODE -DCCD_IDEDOUBLE -DdLIBCCD_ENABLED -DdTHREADING_INTF_DISABLED \
	-DRAPIDXML_NO_EXCEPTIONS -DBYTE_ORDER=LITTLE_ENDIAN -DHAVE_MALLOC_H -DUSE_UNIX_ERRORS
	LDFLAGS  = -static 
	CXX      = g++
	CC       = gcc
	LIBS = -lpthread -lm
	INC_DIRS = -Irapidxml-1.13 -Iexprtk -Iode-0.15/ode/src -Iode-0.15/libccd/src -Iode-0.15/OPCODE -Iode-0.15/include -Iann_1.1.2/include
endif

ifeq ($(HOST),submitter.itservices.manchester.ac.uk)
        #CXXFLAGS = -static -ffast-math -O3 -DEXPERIMENTAL
        CXXFLAGS = -static -O3 -ffast-math -DEXPERIMENTAL \
        -DdIDEDOUBLE -DTRIMESH_ENABLED -DTRIMESH_OPCODE -DCCD_IDEDOUBLE -DdLIBCCD_ENABLED -DdTHREADING_INTF_DISABLED \
        -DRAPIDXML_NO_EXCEPTIONS -DBYTE_ORDER=LITTLE_ENDIAN -DHAVE_MALLOC_H -DUSE_UNIX_ERRORS
        LDFLAGS  = -static 
        # condor_compile is tricky with pthread so best avoided and just use vanilla universe
        # CXX      = condor_compile g++
        # CC       = condor_compile gcc
        CXX      = g++
        CC       = gcc
        LIBS = -lpthread -lm
        INC_DIRS = -Irapidxml-1.13 -Iexprtk -Iode-0.15/ode/src -Iode-0.15/libccd/src -Iode-0.15/OPCODE -Iode-0.15/include -Iann_1.1.2/include
endif

ifeq ($(HOST),l-u-roboticssuite.it.manchester.ac.uk)
        #CXXFLAGS = -static -ffast-math -O3 -DEXPERIMENTAL
        CXXFLAGS = -O3 -ffast-math -DEXPERIMENTAL \
        -DdIDEDOUBLE -DTRIMESH_ENABLED -DTRIMESH_OPCODE -DCCD_IDEDOUBLE -DdLIBCCD_ENABLED -DdTHREADING_INTF_DISABLED \
        -DRAPIDXML_NO_EXCEPTIONS -DBYTE_ORDER=LITTLE_ENDIAN -DHAVE_MALLOC_H -DUSE_UNIX_ERRORS
        LDFLAGS  = 
        # condor_compile is tricky with pthread so best avoided and just use vanilla universe
        # CXX      = condor_compile g++
        # CC       = condor_compile gcc
        CXX      = g++
        CC       = gcc
        LIBS = -lpthread -lm
        INC_DIRS = -Irapidxml-1.13 -Iexprtk -Iode-0.15/ode/src -Iode-0.15/libccd/src -Iode-0.15/OPCODE -Iode-0.15/include -Iann_1.1.2/include
endif

# vpath %.cpp src
# vpath %.c src 

GAITSYMSRC = \
AMotorJoint.cpp\
BallJoint.cpp\
Body.cpp\
BoxCarDriver.cpp\
BoxGeom.cpp\
ButterworthFilter.cpp\
CappedCylinderGeom.cpp\
Contact.cpp\
Controller.cpp\
CyclicDriver.cpp\
CylinderWrapStrap.cpp\
DampedSpringMuscle.cpp\
DataFile.cpp\
DataTarget.cpp\
DataTargetQuaternion.cpp\
DataTargetScalar.cpp\
DataTargetVector.cpp\
Drivable.cpp\
Driver.cpp\
Environment.cpp\
ErrorHandler.cpp\
Face.cpp\
FacetedBox.cpp\
FacetedCappedCylinder.cpp\
FacetedConicSegment.cpp\
FacetedObject.cpp\
FacetedPolyline.cpp\
FacetedRect.cpp\
FacetedSphere.cpp\
FEC.cpp\
Filter.cpp\
FixedDriver.cpp\
FixedJoint.cpp\
FloatingHingeJoint.cpp\
Geom.cpp\
GLUtils.cpp\
HingeJoint.cpp\
Joint.cpp\
MAMuscleComplete.cpp\
MAMuscle.cpp\
MAMuscleExtended.cpp\
Marker.cpp\
MD5.cpp\
MovingAverage.cpp\
Muscle.cpp\
NamedObject.cpp\
NPointStrap.cpp\
ObjectiveMain.cpp\
PCA.cpp\
PIDErrorInController.cpp\
PIDMuscleLength.cpp\
PIDTargetMatch.cpp\
PlaneGeom.cpp\
PositionReporter.cpp\
RayGeom.cpp\
Reporter.cpp\
Simulation.cpp\
SliderJoint.cpp\
SphereGeom.cpp\
StackedBoxCarDriver.cpp\
StepDriver.cpp\
Strap.cpp\
StrokeFont.cpp\
SwingClearanceAbortReporter.cpp\
TCP.cpp\
TegotaeDriver.cpp\
ThreePointStrap.cpp\
TIFFWrite.cpp\
TorqueReporter.cpp\
TrimeshGeom.cpp\
TwoCylinderWrapStrap.cpp\
TwoPointStrap.cpp\
UDP.cpp\
UGMMuscle.cpp\
UniversalJoint.cpp\
Util.cpp\
Warehouse.cpp\
XMLConverter.cpp

LIBCCDSRC = \
alloc.c \
ccd.c \
mpr.c \
polytope.c \
support.c \
vec3.c

ODESRC = \
array.cpp \
box.cpp \
capsule.cpp \
collision_convex_trimesh.cpp \
collision_cylinder_box.cpp \
collision_cylinder_plane.cpp \
collision_cylinder_sphere.cpp \
collision_cylinder_trimesh.cpp \
collision_kernel.cpp \
collision_libccd.cpp \
collision_quadtreespace.cpp \
collision_sapspace.cpp \
collision_space.cpp \
collision_transform.cpp \
collision_trimesh_box.cpp \
collision_trimesh_ccylinder.cpp \
collision_trimesh_disabled.cpp \
collision_trimesh_distance.cpp \
collision_trimesh_gimpact.cpp \
collision_trimesh_opcode.cpp \
collision_trimesh_plane.cpp \
collision_trimesh_ray.cpp \
collision_trimesh_sphere.cpp \
collision_trimesh_trimesh_new.cpp \
collision_trimesh_trimesh.cpp \
collision_util.cpp \
convex.cpp \
cylinder.cpp \
error.cpp \
export-dif.cpp \
fastdot.cpp \
fastldlt.cpp \
fastlsolve.cpp \
fastltsolve.cpp \
heightfield.cpp \
lcp.cpp \
mass.cpp \
mat.cpp \
matrix.cpp \
memory.cpp \
misc.cpp \
nextafterf.c \
objects.cpp \
obstack.cpp \
ode.cpp \
odeinit.cpp \
odemath.cpp \
odeou.cpp \
odetls.cpp \
plane.cpp \
quickstep.cpp \
ray.cpp \
rotation.cpp \
sphere.cpp \
step.cpp \
threading_base.cpp \
threading_impl.cpp \
threading_pool_posix.cpp \
threading_pool_win.cpp \
timer.cpp \
util.cpp 

ODEJOINTSSRC = \
amotor.cpp \
ball.cpp \
contact.cpp \
dball.cpp \
dhinge.cpp \
fixed.cpp \
floatinghinge.cpp \
hinge.cpp \
hinge2.cpp \
joint.cpp \
lmotor.cpp \
null.cpp \
piston.cpp \
plane2d.cpp \
pr.cpp \
pu.cpp \
slider.cpp \
transmission.cpp \
universal.cpp

OPCODEICESRC = \
IceAABB.cpp \
IceContainer.cpp \
IceHPoint.cpp \
IceIndexedTriangle.cpp \
IceMatrix3x3.cpp \
IceMatrix4x4.cpp \
IceOBB.cpp \
IcePlane.cpp \
IcePoint.cpp \
IceRandom.cpp \
IceRay.cpp \
IceRevisitedRadix.cpp \
IceSegment.cpp \
IceTriangle.cpp \
IceUtils.cpp

OPCODESRC = \
OPC_AABBCollider.cpp \
OPC_AABBTree.cpp \
OPC_BaseModel.cpp \
OPC_Collider.cpp \
OPC_Common.cpp \
OPC_HybridModel.cpp \
OPC_LSSCollider.cpp \
OPC_MeshInterface.cpp \
OPC_Model.cpp \
OPC_OBBCollider.cpp \
OPC_OptimizedTree.cpp \
OPC_Picking.cpp \
OPC_PlanesCollider.cpp \
OPC_RayCollider.cpp \
OPC_SphereCollider.cpp \
OPC_TreeBuilders.cpp \
OPC_TreeCollider.cpp \
OPC_VolumeCollider.cpp \
Opcode.cpp \
StdAfx.cpp

ANNSRC = \
ANN.cpp \
bd_fix_rad_search.cpp \
bd_pr_search.cpp \
bd_search.cpp \
bd_tree.cpp \
brute.cpp \
kd_dump.cpp \
kd_fix_rad_search.cpp \
kd_pr_search.cpp \
kd_search.cpp \
kd_split.cpp \
kd_tree.cpp \
kd_util.cpp \
perf.cpp 

GAITSYMOBJ = $(addsuffix .o, $(basename $(GAITSYMSRC) ) )
GAITSYMHEADER = $(addsuffix .h, $(basename $(GAITSYMSRC) ) ) PGDMath.h DebugControl.h SimpleStrap.h

LIBCCDOBJ = $(addsuffix .o, $(basename $(LIBCCDSRC) ) )
ODEOBJ = $(addsuffix .o, $(basename $(ODESRC) ) )
ODEJOINTSOBJ = $(addsuffix .o, $(basename $(ODEJOINTSSRC) ) )
OPCODEICEOBJ = $(addsuffix .o, $(basename $(OPCODEICESRC) ) )
OPCODEOBJ = $(addsuffix .o, $(basename $(OPCODESRC) ) )
ANNOBJ = $(addsuffix .o, $(basename $(ANNSRC) ) )

BINARIES = bin/gaitsym bin/gaitsym_udp bin/gaitsym_tcp

all: directories binaries 

directories: bin obj 

binaries: $(BINARIES)

obj: 
	-mkdir obj
	-mkdir obj/cl
	-mkdir obj/udp
	-mkdir obj/tcp
	-mkdir obj/libccd
	-mkdir obj/ode
	-mkdir obj/odejoints
	-mkdir obj/opcodeice
	-mkdir obj/opcode
	-mkdir obj/ann 

bin:
	-mkdir bin

obj/cl/%.o : src/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

obj/libccd/%.o : ode-0.15/libccd/src/%.c
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

obj/ode/%.o : ode-0.15/ode/src/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

obj/ode/%.o : ode-0.15/ode/src/%.c
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

obj/odejoints/%.o : ode-0.15/ode/src/joints/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

obj/opcodeice/%.o : ode-0.15/OPCODE/Ice/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

obj/opcode/%.o : ode-0.15/OPCODE/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

obj/ann/%.o :  ann_1.1.2/src/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@


bin/gaitsym: $(addprefix obj/cl/, $(GAITSYMOBJ) ) $(addprefix obj/libccd/, $(LIBCCDOBJ) ) $(addprefix obj/ode/, $(ODEOBJ) ) \
$(addprefix obj/odejoints/, $(ODEJOINTSOBJ) ) $(addprefix obj/opcodeice/, $(OPCODEICEOBJ) ) $(addprefix obj/opcode/, $(OPCODEOBJ) ) \
$(addprefix obj/ann/, $(ANNOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

obj/tcp/%.o : src/%.cpp
	$(CXX) -DUSE_TCP $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

bin/gaitsym_tcp: $(addprefix obj/tcp/, $(GAITSYMOBJ) ) $(addprefix obj/libccd/, $(LIBCCDOBJ) ) $(addprefix obj/ode/, $(ODEOBJ) ) \
$(addprefix obj/odejoints/, $(ODEJOINTSOBJ) ) $(addprefix obj/opcodeice/, $(OPCODEICEOBJ) ) $(addprefix obj/opcode/, $(OPCODEOBJ) ) \
$(addprefix obj/ann/, $(ANNOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(SOCKET_LIBS) $(LIBS)

obj/udp/%.o : src/%.cpp
	$(CXX) -DUSE_UDP $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

bin/gaitsym_udp: $(addprefix obj/udp/, $(GAITSYMOBJ) ) $(addprefix obj/libccd/, $(LIBCCDOBJ) ) $(addprefix obj/ode/, $(ODEOBJ) ) \
$(addprefix obj/odejoints/, $(ODEJOINTSOBJ) ) $(addprefix obj/opcodeice/, $(OPCODEICEOBJ) ) $(addprefix obj/opcode/, $(OPCODEOBJ) ) \
$(addprefix obj/ann/, $(ANNOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(UDP_LIBS) $(LIBS)


clean:
	rm -rf obj bin
	rm -rf distribution
	rm -rf build*

superclean:
	rm -rf obj bin
	rm -rf distribution
	rm -rf build*
	rm -rf GaitSymQt/GaitSymQt.pro.user.*
	find . -name '.DS_Store' -exec rm -f {} \;
	find . -name '.gdb_history' -exec rm -f {} \;
	find . -name '.#*' -exec rm -f {} \;
	find . -name '*~' -exec rm -f {} \;
	find . -name '#*' -exec rm -f {} \;
	find . -name '*.bak' -exec rm -f {} \;
	find . -name '*.bck' -exec rm -f {} \;
	find . -name '*.tmp' -exec rm -f {} \;
	find . -name '*.o' -exec rm -f {} \;

distribution: distribution_dirs gaitsym_distribution gaitsym_distribution_extras

distribution_dirs:
	rm -rf distribution
	-mkdir distribution
	-mkdir distribution/src

gaitsym_distribution: $(addprefix distribution/src/, $(GAITSYMSRC)) $(addprefix distribution/src/, $(GAITSYMHEADER))

$(addprefix distribution/src/, $(GAITSYMSRC)):
	scripts/strip_ifdef.py EXPERIMENTAL $(addprefix src/, $(notdir $@)) $@ 

$(addprefix distribution/src/, $(GAITSYMHEADER)):
	scripts/strip_ifdef.py EXPERIMENTAL $(addprefix src/, $(notdir $@)) $@ 

gaitsym_distribution_extras:
	cp -rf ann_1.1.2 distribution/
	cp -rf exprtk distribution/
	cp -rf GaitSymQt distribution/
	cp -rf irrlicht-1.9 distribution/
	cp -rf libgwavi distribution/
	cp -rf ode-0.15 distribution/
	cp -rf rapidxml-1.13 distribution/
	cp makefile distribution/
	find distribution -depth -type d -name CVS -print -exec rm -rf {} \;
	rm -rf distribution/GaitSymQt/GaitSymQt.pro.*


