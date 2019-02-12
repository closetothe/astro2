# Retrieved from https://github.com/johnny-wang/odroid-install

printf "\nInstalling ROS\n\n"
printf "\nEnabling universe and multiverse repos\n\n"

sudo add-apt-repository universe && sudo apt-get update
sudo add-apt-repository multiverse && sudo apt-get update

printf "\nSetting up locale\n\n"
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

printf "\nSetting up sources\n\n"
# (meant for Ubuntu 14.04 but seems to work anyway)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


printf "Setting up keys"
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update

prinf "\nDoing the rest...\n\n"
sudo apt-get --yes install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

printf "\n\n\n----DONE----\n\n\n"



