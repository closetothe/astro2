sudo apt-get -y install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-ps3joy ros-kinetic-joystick-drivers ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard

source ~/.bashrc

printf "\n\n--------------------------\nSetting up catkin workspace at ~/ws\n--------------------------\n\n"
mkdir -p ~/ws/src
cd ~/ws
catkin_make

printf "\n\n--------------------------\nCatkin workspace created at ~/ws\n--------------------------\n\n"


./install_ros_differential_drive.sh
