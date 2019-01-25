sudo chmod +x *

if [ ! -d arduino ]
then
	sudo ./install_core.sh
	sudo ./install_arduino.sh		
	sudo ./install_opencv.sh
	sudo ./install_ros.sh
	sudo ./install_ros_deps.sh
else
	printf "\n'arduino' folder not found. Please download the arduino installation folder, put it here, and rename it as 'arduino'.\n"
