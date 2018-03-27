
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

OS = LINUX
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

process_tools: slcrunch slcalibrate plotlens plyalign plytrim chessgen chessfind listcreator
capture_tools: sldisp

all: process_tools capture_tools

bin:
	@mkdir -p bin

slturn: bin slturn.cpp hid.o
	$(CC) $(CFLAGS) $(CLIB) $(SYSARG) $(HID) $(FLAGS_OSC) slturn.cpp -o bin/slturn

sldisp: bin sldisp.cpp util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_GL) sldisp.cpp util.cpp -o bin/sldisp $(FLAGS_SDL) $(FLAGS_OSC)

slcrunch: bin slcrunch.cpp util.o sl_util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) slcrunch.cpp util.cpp sl_util.cpp -o bin/slcrunch

slcalibrate: bin slcalibrate.cpp util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) slcalibrate.cpp util.cpp -o bin/slcalibrate

plotlens: bin plotlens.cpp util.o sl_util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) plotlens.cpp util.cpp sl_util.cpp -o bin/plotlens

plyalign: bin plyalign.cpp util.o sl_util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_VTK) $(FLAGS_EIGEN) $(FLAGS_PCL) $(FLAGS_OPENCV) $(FLAGS_FLANN) $(FLAGS_BOOST) plyalign.cpp util.cpp sl_util.cpp -o bin/plyalign

plytrim: bin plytrim.cpp util.o sl_util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_VTK) $(FLAGS_EIGEN) $(FLAGS_PCL) $(FLAGS_OPENCV) $(FLAGS_FLANN) $(FLAGS_BOOST) plytrim.cpp util.cpp sl_util.cpp -o bin/plytrim

plymerge: bin plymerge.cpp util.o sl_util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_EIGEN) $(FLAGS_PCL) $(FLAGS_OPENCV) $(FLAGS_FLANN) $(FLAGS_BOOST) plymerge.cpp util.cpp sl_util.cpp -o bin/plymerge

chessgen: bin chessgen.cpp util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) chessgen.cpp util.cpp -o bin/chessgen

chessfind: bin chessfind.cpp util.o
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) chessfind.cpp util.cpp -o bin/chessfind
	
listcreator: bin listcreator.cpp
	$(CC) $(CFLAGS) $(CLIB) $(FLAGS_OPENCV) listcreator.cpp -o bin/listcreator

#hid.o: hid_$(OS).c hid.h
hid.o: $(HID) hid.h
	$(CC) $(CFLAGS) -c -o $@ $<

util.o: util.cpp util.h
	$(CC) -c -o util.o util.cpp

sl_util.o: sl_util.cpp sl_util.h
	$(CC) -c -o sl_util.o sl_util.cpp

clean:
	rm util.o sl_util.o
	rm bin/slturn bin/sldisp bin/slcrunch bin/slcalibrate bin/plotlens bin/plyalign bin/plytrim bin/chessgen bin/chessfind bin/listcreator
	rmdir bin

