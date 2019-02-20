#!/bin/bash

DOWNLOADS="~/Downloads"
DESKTOP="~/Desktop"
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
	printf "\n\e[34mRealSense 1.* library not found. Installing now...\e[0m\n\n"
	apt-get install libusb-1.0-0-dev libglfw3-dev
	
	if [ -d $DOWNLOADS/librealsense* ] || [ -e $DOWNLOADS/librealsense* ]
	then
		rm -rf $DOWNLOADS/librealsense*
	fi

	wget -O $DOWNLOADS/librealsense.zip https://github.com/IntelRealSense/librealsense/archive/v1.12.1.zip
	
	if [ -d $DESKTOP/librealsense1 ]
	then
		rm -rf $DESKTOP/librealsense1
	fi
	unzip $DOWNLOADS/librealsense.zip -d $DOWNLOADS/librealsense
	mv $DOWNLOADS/librealsense/librealsense-1* $DESKTOP/librealsense1
	rm -rf $DOWNLOADS/librealsense	
	mkdir $DESKTOP/librealsense1/build
	cd $DESKTOP/librealsense1/build
	cmake .. -DBUILD_EXAMPLES:BOOL=true
	make && sudo make install && printf "\n\e[32mSuccessfully installed RealSense SDK 1.12\e[0m\n\n"
	printf "\nPatching uvc video drivers...\n\n"
	bash $DESKTOP/librealsense1/scripts/patch-uvcvideo-16.04.simple.sh
	modprobe uvcvideo
	printf "\n\e[34mRS example demos have been installed to /usr/local/bin\e[0m\n\n"
else
	printf "\n\e[34mRealSense library is already installed.\e[0m\n\n"
fi

