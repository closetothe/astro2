#!/bin/bash

DOWNLOADS="/home/odroid/Downloads"
DESKTOP="/home/odroid/Desktop"
RS_DIR="/usr/local/include/librealsense"
GLFW_DIR="/usr/local/include/GLFW"
RSLIB_PATH="/usr/local/lib/librealsense*"
GLFWLIB_PATH="/usr/local/lib/libglfw*"
CV_DIR="/usr/local/include/opencv*"
CVLIB_PATH="/usr/local/lib/libopencv*"
EIGEN_DIR="/usr/local/include/eigen3"
BOOST_PATH="/usr/local/lib/libboost*"
BOOST_DIR="/usr/local/include/boost*"
FLANN_PATH="/usr/local/lib/libflann*"
FLANN_DIR="/usr/local/include/flann*"
VTK_PATH="/usr/local/lib/libvtk*"
VTK_DIR="/usr/local/include/vtk*"
PCL_PATH="/usr/local/lib/libpcl*"
PCL_DIR="/usr/local/include/pcl*"
clear

# RealSense Library
if [ ! -d $RS_DIR ] || [ ! -e $RSLIB_PATH ]
then
	printf "\nRealSense library not found. Installing now...\n\n"
	apt-get install libusb-1.0-0-dev libglfw3-dev
	
	if [ -d $DOWNLOADS/librealsense* ] || [ -e $DOWNLOADS/librealsense* ]
	then
		rm -rf $DOWNLOADS/librealsense*
	fi

	wget -O $DOWNLOADS/librealsense.zip https://github.com/IntelRealSense/librealsense/archive/v1.12.1.zip
	
	if [ -d $DESKTOP/librealsense ]
	then
		rm -rf $DESKTOP/librealsense
	fi
	unzip $DOWNLOADS/librealsense.zip -d $DOWNLOADS/librealsense
	mv $DOWNLOADS/librealsense/librealsense-1* $DESKTOP/librealsense
	rm -rf $DOWNLOADS/librealsense	
	mkdir $DESKTOP/librealsense/build
	cd $DESKTOP/librealsense/build
	cmake .. -DBUILD_EXAMPLES:BOOL=true
	make && make install
	printf "\nPatching uvc video drivers...\n\n"
	.$DESKTOP/librealsense/scripts/patch-uvcvideo-16.04.simple.sh
	modprobe uvcvideo
	printf "\nRS example demos have been installed to /usr/local/bin\n\n"
else
	printf "\nRealSense library is already installed.\n\n"
fi

