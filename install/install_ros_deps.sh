sudo apt-get -y install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-ps3joy ros-kinetic-joystick-drivers ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-depthimage-to-laserscan

source ~/.bashrc

printf "\n\e[34mSetting up ROS Arduino library at ~/Arduino/libraries\e[0m\n\n"
cd ~/Arduino/libraries && rm -rf ros_lib && rosrun rosserial_arduino make_libraries.py . && printf "\n\e[32mInstalled ROS Arduino library at ~/Arduino/libraries\e[0m\n\n"

printf "\n\e[34mSetting up catkin workspace at ~/ws\e[0m\n\n"
mkdir -p ~/ws/src
cd ~/ws
source ~/.bashrc && catkin_make && echo "source ~/ws/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc && printf "\n\e[32mCatkin workspace created at ~/ws\e[0m\n\n"
