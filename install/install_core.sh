clear

printf "\n\e[34mEnabling auto-login for headless SSH to work\e[0m\n\n"
su
sudo echo "autologin-user=odroid" >>  /usr/share/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf
su odroid

printf "\n\e[34mCreating 4GB swap space\e[0m\n\n"
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

printf "\n\e[34mInstalling a bunch of packages...\e[0m\n\n"
sudo apt-get update
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily.timer

# sudo apt-get -y install linux-firmware
sudo apt-get -y install libc6-dev build-essential dkms git cmake i2c-tools python-smbus dconf-tools doxygen pkg-config wget curl unzip x11vnc xorg openbox evince xboxdrv

printf "\n\e[34mInstalling Odroid Utility\e[0m\n\n"
sudo wget -O /usr/local/bin/odroid-utility.sh https://raw.githubusercontent.com/mdrjr/odroid-utility/master/odroid-utility.sh
sudo chmod +x /usr/local/bin/odroid-utility.sh
source ~/.bashrc

printf "\n\e[34mGiving executable permissions to installation scripts\e[0m\n\n"
sudo chmod +x *.sh

printf "\n\e[34mInstalling ncurses library\e[0m\n\n"
sudo apt-get --yes install libncurses5-dev libncursesw5-dev

printf "\n\e[34mInstalling Python\e[0m\n\n"
sudo apt-get --yes install python-dev python-pip python3-dev python3-pip

## NOT WORKING ##
# printf "\nConfiguring WiFi dongle with Realtek 8821cu driver"
# cd wifisetup
# make
# sudo make install

















