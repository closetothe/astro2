sudo apt-get -y install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-ps3joy ros-kinetic-joystick-drivers ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard

source ~/.bashrc

printf "\n\e[34mSetting up catkin workspace at ~/ws\e[0m\n\n"
mkdir -p ~/ws/src
cd ~/ws
source ~/.bashrc
catkin_make
echo "source ~/ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
printf "\n\e[32mCatkin workspace created at ~/ws\e[0m\n\n"
