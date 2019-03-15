# Jamiel Rahi 2019
# github.com/closetothe/
# Concordia University MIE
# ASTRO 2 Capstone Project

###############################
## TESTED ONLY ON ODROID XU4 ##
###############################

# This script installs basic packages for a clean-slate Raspberry Pi 3 B
# running Ubuntu MATE 16.04

$USER = astropi

clear

printf "\n\e[34mEnabling auto-login for headless SSH to work\e[0m\n\n"

sudo bash -c 'echo "autologin-user=astropi" >>  /usr/share/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf'


printf "\n\e[34mCreating 2GB swap space\e[0m\n\n"
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

printf "\n\e[34mInstalling a bunch of packages...\e[0m\n\n"
sudo apt-get update
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily.timer

sudo apt-get -y install libc6-dev build-essential dkms git cmake i2c-tools python-smbus dconf-tools vim doxygen pkg-config wget curl unzip x11vnc xorg openbox evince xboxdrv

printf "\n\e[34mInstalling Python\e[0m\n\n"
sudo apt-get --yes install python-dev python-pip python3-dev python3-pip
