## Synopsis

The main goal of structured light scanning is to calculate the dense correspondence between camera pixels and output pixels. By displaying black and white patterns to signal 0 and 1, a flat panel display or projector can simply "speak in binary" the pixel coordinates to the camera. Multiple patterns are shown to signal each bit of the per-pixel address code (e.g. x output pixel address equals 10 -> binary 1010 so white-black-white-black is displayed at that output device pixel). The process of decoding images of displayed black-white patterns is executed as a simple rule for all camera pixels (was pixel bright or not).

Binary gray code structured light scanning involves displaying, capturing, and parsing sequences of black and white patterns. Uses include camera lens calibration and 3D point cloud reconstruction. By displaying specific patterns (on a display or projector) and capturing images of them, it is possible to calculate the correspondence between camera pixels and outut device pixels. More details available at https://en.wikipedia.org/wiki/Correspondence_problem https://en.wikipedia.org/wiki/Structured_Light_3D_Scanner

## sldisp

Tool for displaying binary gray code patterns using an output device (e.g. LCD screen, DLP projector). Receives pattern change instructions over network (default localhost). Pattern input is OSC packets /sl/iii (int,int,int) for (bit,axis,inverted) 0=false, 1=true

## slcrunch

A tool for processing image sequences of binary gray code patterns into correspondence data or 3D point clouds. Takes a list of images as input. See listcreator for format of input file list.

## slcalibrate

A tool for camera and projector lens calibration, as well as camera-projector registration. For camera lens calibration, a flatscreen display is used to show patterns and only camera~display correspondence data is required. For projector-camera calibration, a camera, projector, and a flat printed chessboard pattern is needed (see slcrunch -help for details).

## chessgen

A tool for generating images of chess patterns for lens calibration. Specify edge length in pixels, number of x/y inner corners, and x/y image output dimensions.

## plotlens

This program creates an image visualizing image distortion using calibration data input. Helpful for understanding calibration data.

## plyalign

(WIP) Tool for automatically aligning multiply .PLY 3D pointcloud files. Specify .PLY files in sequential order, and repeated rotation from scan to scan is estimated.

## plytrim

(WIP) Tool for trimming .PLY 3D pointcloud files. Specify center coordinates and dimensions of 3D axis-aligned bounding box.

## listcreator

This program creates a file containing a list of images in a sequence. Since image sequences need to be processed as a whole, image filename lists are used to pass them to slcrunch and slcalibrate. Capture programs should generate lists automatically, as in guppy_cap.cpp.

## Prerequisites

Need installed:
opencv2 pcl2 liblo sdl2 flann boost

## Building

Built using: OpenCV(2.4.10), SDL2(2.0.3), liblo(0.28), exiftool(9.69), on Mac OS X 10.10.2, 10.11.6, 10.12

Some programs link with OpenCV, an open source image processing library <http://opencv.org/>.
slcrunch calls the exiftool program for reading image metadata. <http://www.sno.phy.queensu.ca/~phil/exiftool/>
sldisp links with SDL2, a user interface library. More information at <https://www.libsdl.org/>.
Some programs with liblo, an Open Sound Control library for packet communication. <http://opensoundcontrol.org/implementations>

I have a basic Makefile; it should compile all the tools with "make" or individual ones with a target e.g. "make slcrunch".

## Use

sltk is comprised of various standalone command-line programs to handle separate tasks for structured light scanning. For capturing, sldisp is run on the computer connected to the output device (e.g. monitor, projector) in order to show structured light patterns. A corresponding capture program (e.g. slcapture.py) is run on the computer connected to the input device (e.g. firewire/usb camera). By default, both computers are the same and sldisp/guppy_cap communicate over localhost via OSC UDP packets.

Typical lens calibration consists of a camera-monitor (flatscreen display) setup, with sldisp showing structured light patterns on the display. guppy_cap is used to capture image sequences, which are processed into camera-display correspondence data with slcrunch. Once you've collected enough different viewpoints of a flatscreen monitor, the correspondence data is passed to slcalibrate. Calibration output from slcalibrate includes camera matrix, camera distortion coefficients, and camera pixel dimensions.

Typical projector calibration consists of a camera-projector setup, with sldisp showing structured light patterns on the projector. guppy_cap is used to capture image sequences of a flat chess calibration pattern, which are processed into camera-chess-projector correspondence data with slcrunch  Once you've collected enough different viewpoints of the chess pattern, the correspondence data is passed to slcalibrate. Calibration output includes camera matrix, camera distortion, projector matrix, projector distortion, etc. Separate calibration of lens and projector is strongly recomended.

Calibration output can be visualized using plotlens to see a color mapped distortion image.

slcalibrate can take previous calibration files for camera or projector, to reuse intrinsic calibration data. Used for camera-projector registration.


## RPI example


