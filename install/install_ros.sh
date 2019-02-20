# Retrieved from https://github.com/johnny-wang/odroid-install

printf "\n\e[34mInstalling ROS\e[0m\n\n"
printf "\n\e[34mEnabling universe and multiverse repos\e[0m\n\n"

sudo add-apt-repository universe && sudo apt-get update
sudo add-apt-repository multiverse && sudo apt-get update

printf "\n\e[34mSetting up locale\e[0m\n\n"
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

printf "\n\e[34mSetting up sources\e[0m\n\n"
# (meant for Ubuntu 14.04 but seems to work anyway)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


printf "\n\e[34mSetting up keys\e[0m\n\n"
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update

prinf "\n\e[34mDoing the rest...\e[0m\n\n"
sudo apt-get --yes install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

printf "\n\e[32mDONE\e[0m\n\n"



