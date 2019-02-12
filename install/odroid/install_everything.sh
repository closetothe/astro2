sudo chmod +x *

if [ -d arduino ]
then
	./install_core.sh && ./install_arduino.sh && ./install_opencv.sh && \
	./install_ros.sh && ./install_ros_deps.sh && \
	./install_ros_differential_drive.sh \
	&& ./install_cmake.sh && ./install_librealsense2.sh \
	&& ./install_ros_realsense2.sh
else
	printf "\n'arduino' folder not found. Please download the arduino installation folder, put it here, and rename it as 'arduino'\n"
fi
