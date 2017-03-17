# SLTK

SLTK stands for Structured Light Took Kit. These are a collection of open source command line and GUI tools I have written with the intention of making operation of DIY structured light systems easier for myself. An effort has been made to separate the tools into logical units of work. It is possible to run the display program, the camera capture program, and the control program all on separate networked computers.

Here's an example of a sea shell generated from 74 aligned pointclouds captured using a raspberry pi v1 camera + sony mp-cl1 projector + arduino-based turntable (~5deg per turn): [https://sketchfab.com/models/5def256988594fae93ceb69b59fabbcc](https://sketchfab.com/models/5def256988594fae93ceb69b59fabbcc)

Two sets of raw+processed data from individual sea shell scans are available in the sampledata directory, along with the associated projector/camera calibration file.

## Uses

* Finding **intrinsic camera calibration**: 3x3 matrix + radial/tangential distortion coefficients
* Finding **intrinsic projector calibration**: 3x3 matrix + radial/tangential distortion coefficients
* Finding **extrinsic matrix** for a stereo camera + projector setup
* Calculating **correspondence map** between camera and display pixels (screen or projector)
* Generating **3D point clouds** from **correspondence data** and **intrinsic+extrinsic calibration data**
* Rendering a **visualization of calibration data** with rainbow color map and isolines of distortion magnitude per pixel
* **Displaying** timed structured light patterns via graphics adapter to display or projector
* **Capturing** timed images of structured light patterns via camera input
* **Automated rotation** of objects using a basic arduino stepper motor turntable
* **Merging multiple point clouds** for generating a static background point set to allow automated cleanup
* **Background subtraction** for cleaning up 3D point clouds by specifying a "background" point set
* **Automated alignment** of 3D point clouds (requires background subtraction to remove static objects)
* **Cultural Heritage Preservation**

## Synopsis

This collection of programs is intended to be used to create dense 3D pointclouds of semi-matte objects.
This toolkit requires hardware in order to scan objects. Hardware tried for development includes:

1) raspberry pi (v3 model b and zero)
2) rpi camera (v1 and v2)
3) flatscreen LCD monitor
4) projector
5) turntable

## mjpeg-streamer [external tool]

This third party program (https://github.com/jacksonliam/mjpg-streamer) continuously captures images from a camera and streams them over the network. A fork is used in this case to specifically ensure support for the rpi camera, although it should be possible to use other cameras. Please see mjpg-streamer docs for more info.
Requires: camera

## sldisp

Tool for generating structured light patterns. Supports binary gray code, sinusoid, and monochrome RGB pattern outputs. Receives OSC messages and updates output pattern.
Requires: display device (e.g. LCD screen, DLP projector), slcapture

## slcapture.py

Main control program: streams images continuously from an mjpg-streamer server while triggering preset pattern sequences on an sldisp client.
Each "scan" output is really just a directory of images, each taken while a specific structured light pattern is displayed. These scans are processed by slcrunch.
Requires: mjpg-streamer, camera, sldisp, display device

## slcrunch

A tool for processing image sequences of e.g. binary gray code patterns into raw correspondence data or 3D point clouds. Requires *accurate* projector-camera calibration file in order to generate useful point clouds. Takes a list of images as input. See sampledata directory for list format.

## slcalibrate

A tool for camera and projector lens calibration, as well as camera-projector registration. For camera lens calibration, a flatscreen display is used to show binary gray code patterns and only camera~display correspondence data is required. For projector-camera calibration, a camera, projector, and a flat printed chessboard pattern is needed (see slcrunch -help for details). See wiki for more info.

## chessgen

A tool for generating images of chess patterns for lens calibration. Specify edge length in pixels, number of x/y inner corners, and x/y image output dimensions.

## plotlens

This program creates an image visualizing image distortion using calibration data input. Helpful for understanding quality of calibration data.

## plyalign

(WIP) Tool for automatically aligning multiple .PLY 3D pointcloud files. Specify .PLY files in sequential order, and repeated rotation from scan to scan is estimated.

## plymerge

Tool for merging multiple .PLY 3D pointcloud files. Combines all input files into one as-is with no alignment performed. Handy for making background subtraction files for speedy cleanup.

## plytrim

Tool for trimming .PLY 3D pointcloud files. Input a primary/reference pointcloud + distance X: all points in primary set within X units of any point in reference are removed. Output file only contains points in primary set which have no neighbors within X units in reference set.

## listcreator

This program creates a file containing a list of images in a sequence. Since image sequences need to be processed as a group, image filename lists are used to pass them to slcrunch and slcalibrate. Capture programs should generate lists automatically, as in slcapture.py. This tool is a manual fallback.

## Prerequisites

Need installed:
opencv2 pcl2 liblo sdl2 flann boost

## Building

Built using: OpenCV(2.4.10), SDL2(2.0.3), liblo(0.28), exiftool(9.69), on Mac OS X 10.10.2, 10.11.6, 10.12

mjpg-streamer is used to ingest camera data <https://github.com/jacksonliam/mjpg-streamer>
Some programs link with OpenCV, an open source image processing library <http://opencv.org/>
slcrunch calls the exiftool program for reading image metadata. <http://www.sno.phy.queensu.ca/~phil/exiftool/>
sldisp links with SDL2, a user interface library. More information at <https://www.libsdl.org/>
Some programs with liblo, an Open Sound Control library for packet communication. <http://opensoundcontrol.org/implementations>

I have prepared a basic Makefile; it should compile all the tools with "make" or individual ones with a target e.g. "make slcrunch". Could use some work.

## Separation of tasks

sltk is split up into standalone command line programs, each meant to handle a separate set of tasks in a structured light processing workflow.

mjpg-streamer is run on a computer connected to a supported camera.
sldisp is run on a computer connected to a flatscreen or projector.
slcapture.py talks to the two programs above to display sequences of patterns and save images of each in a directory
slcrunch processes a directory of images into a correspondence file, or into a 3D point cloud with appropriate calibration input file
slcalibrate processes correspondence files and outputs calibration data for cameras and projectors

Typical lens calibration consists of a camera-monitor (flatscreen display) setup, with sldisp showing structured light patterns on the display. guppy_cap is used to capture image sequences, which are processed into camera-display correspondence data with slcrunch. Once you've collected enough different viewpoints of a flatscreen monitor, the correspondence data is passed to slcalibrate. Calibration output from slcalibrate includes camera matrix, camera distortion coefficients, and camera pixel dimensions.

Typical projector calibration consists of a camera-projector setup, with sldisp showing structured light patterns on the projector. guppy_cap is used to capture image sequences of a flat chess calibration pattern, which are processed into camera-chess-projector correspondence data with slcrunch  Once you've collected enough different viewpoints of the chess pattern, the correspondence data is passed to slcalibrate. Calibration output includes camera matrix, camera distortion, projector matrix, projector distortion, etc. Separate calibration of lens and projector is strongly recomended.

Calibration output can be visualized using plotlens to see a color mapped distortion image.

slcalibrate can take previous calibration files for camera or projector, to reuse intrinsic calibration data. Used for camera-projector registration.


## RPI example hardware


