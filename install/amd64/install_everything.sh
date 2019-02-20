sudo chmod +x *

if [ -d arduino ]
then
	./install_core.sh && ./../install_arduino.sh && ./../install_opencv.sh && \
	./../install_opengl && ./../install_ros.sh && source ~/.bashrc && \
	./../install_ros_deps.sh && ./install_librealsense2.sh && source ~/.bashrc && \
	./install_ros_realsense2.sh
else
	printf "\n'arduino' folder not found. Please download the arduino installation folder, put it here, and rename it as 'arduino'\n"
fi
