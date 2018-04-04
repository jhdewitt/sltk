
# chessfind.cpp
# chessgen.cpp
# listcreator.cpp
# plotlens.cpp
# plyalign.cpp
# plytrim.cpp
# slcalibrate.cpp
# slcrunch.cpp
# sldisp.cpp
# slturn.cpp

#OS = LINUX
#OS = MACOSX
#OS = WINDOWS

CC=g++
CFLAGS=#-Wall -O3
CLIB=-lstdc++
LDFLAGS=
SOURCES=
FLAGS_FLANN=`pkg-config --libs --cflags flann`
FLAGS_OPENCV=`pkg-config --libs --cflags opencv`
FLAGS_BOOST=-lboost_system
FLAGS_VTK=-I/usr/local/Cellar/vtk/7.0.0_6/include/vtk-7.0 -L/usr/local/Cellar/vtk/7.0.0_6/lib
PCL_V=1.8
PCL_MODULES=pcl_common-$(PCL_V) pcl_io-$(PCL_V) pcl_kdtree-$(PCL_V) pcl_registration-$(PCL_V) pcl_segmentation-$(PCL_V) pcl_visualization-$(PCL_V) pcl_search-$(PCL_V)
FLAGS_PCL=`pkg-config --libs --cflags $(PCL_MODULES)`
FLAGS_EIGEN=`pkg-config --libs --cflags eigen3`
FLAGS_OSC=`pkg-config --libs --cflags liblo`
FLAGS_SDL=`sdl2-config --static-libs --cflags`

FLAGS_GL=

ifeq ($(OS),Windows_NT)
	
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
		FLAGS_GL=-lGL -lGLU -lglut
		SYSARG=-lusb
		HID=hid_LINUX.c
    endif
    ifeq ($(UNAME_S),Darwin)
		#CC="clang++"
		#CLIB="-std=c++11"
		#CLIB="-std=c++0x"
		FLAGS_GL=-framework OpenGL
		OSXSDK=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk
		SYSARG=-isysroot $(OSXSDK) -framework IOKit -framework CoreFoundation
		HID=hid_MACOSX.c
    endif
endif

main_tools: sldisp slcrunch slcalibrate plotlens chessgen chessfind listcreator
ply_tools: plyalign plytrim plymerge
	
all: main_tools

bin:
	@mkdir -p bin

slturn: bin slturn.cpp hid.o
	$(CC) $(CFLAGS) $(CLIB) $(SYSARG) $(HID) $(FLAGS_OSC) slturn.cpp -o bin/slturn

sldisp: bin sldisp.cpp util.o
	$(CC) $(CFLAGS) $(CLIB) sldisp.cpp util.cpp -o bin/sldisp $(FLAGS_SDL) $(FLAGS_OSC) $(FLAGS_GL)

slcrunch: bin slcrunch.cpp util.o sl_util.o
	$(CC) slcrunch.cpp util.cpp sl_util.cpp -o bin/slcrunch $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) 

slcalibrate: bin slcalibrate.cpp util.o
	$(CC) slcalibrate.cpp util.cpp -o bin/slcalibrate $(CFLAGS) $(CLIB) $(FLAGS_OPENCV)

plotlens: bin plotlens.cpp util.o sl_util.o
	$(CC) plotlens.cpp util.cpp sl_util.cpp -o bin/plotlens $(CFLAGS) $(CLIB) $(FLAGS_OPENCV)

plyalign: bin plyalign.cpp util.o sl_util.o
	$(CC) plyalign.cpp util.cpp sl_util.cpp -o bin/plyalign $(CFLAGS) $(CLIB) $(FLAGS_VTK) $(FLAGS_EIGEN) $(FLAGS_PCL) $(FLAGS_OPENCV) $(FLAGS_FLANN) $(FLAGS_BOOST)

plytrim: bin plytrim.cpp util.o sl_util.o
	$(CC) plytrim.cpp util.cpp sl_util.cpp -o bin/plytrim $(CFLAGS) $(CLIB) $(FLAGS_VTK) $(FLAGS_EIGEN) $(FLAGS_PCL) $(FLAGS_OPENCV) $(FLAGS_FLANN) $(FLAGS_BOOST) 

plymerge: bin plymerge.cpp util.o sl_util.o
	$(CC) plymerge.cpp util.cpp sl_util.cpp -o bin/plymerge $(CFLAGS) $(CLIB) $(FLAGS_EIGEN) $(FLAGS_PCL) $(FLAGS_OPENCV) $(FLAGS_FLANN) $(FLAGS_BOOST)

chessgen: bin chessgen.cpp util.o
	$(CC) chessgen.cpp util.cpp -o bin/chessgen $(CFLAGS) $(CLIB) $(FLAGS_OPENCV)

chessfind: bin chessfind.cpp util.o
	$(CC) chessfind.cpp util.cpp -o bin/chessfind $(CFLAGS) $(CLIB) $(FLAGS_OPENCV)
	
listcreator: bin listcreator.cpp
	$(CC) listcreator.cpp -o bin/listcreator $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) 

#hid.o: hid_$(OS).c hid.h
hid.o: $(HID) hid.h
	$(CC) $(CFLAGS) -c -o $@ $<

util.o: util.cpp util.h
	$(CC) -c -o util.o util.cpp

sl_util.o: sl_util.cpp sl_util.h
	$(CC) -c -o sl_util.o sl_util.cpp

clean:
	rm -f util.o sl_util.o
	rm -f bin/slturn bin/sldisp bin/slcrunch bin/slcalibrate bin/plotlens bin/plyalign bin/plytrim bin/chessgen bin/chessfind bin/listcreator
	rmdir bin

