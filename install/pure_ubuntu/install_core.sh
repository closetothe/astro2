clear

printf "\n\e[34mInstalling a bunch of packages...\e[0m\n\n"
sudo apt-get update
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily.timer
sudo systemctl disable apt-daily-upgrade.timer
sudo systemctl disable apt-daily-upgrade.service

sudo apt-get -y install libc6-dev build-essential dkms git cmake i2c-tools python-smbus dconf-tools vim doxygen pkg-config wget curl unzip x11vnc xorg openbox evince xboxdrv

printf "\n\e[34mGiving executable permissions to installation scripts\e[0m\n\n"
sudo chmod +x *.sh

printf "\n\e[34mInstalling ncurses library\e[0m\n\n"
sudo apt-get --yes install libncurses5-dev libncursesw5-dev

printf "\n\e[34mInstalling Python\e[0m\n\n"
sudo apt-get --yes install python-dev python-pip python3-dev python3-pip
















