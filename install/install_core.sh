clear

printf "\nEnabling auto-login for headless SSH to work\n\n"
sudo echo "autologin-user=odroid" >>  /usr/share/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf

printf "\nCreating 4GB swap space\n\n"
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

"\nInstalling a bunch of packages...\n\n"
sudo apt-get update
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily.timer

# sudo apt-get -y install linux-firmware
sudo apt-get -y install libc6-dev build-essential dkms git cmake i2c-tools python-smbus dconf-tools doxygen pkg-config wget curl unzip x11vnc xorg openbox evince xboxdrv

printf "\nInstalling Odroid Utility\n\n"
wget -O /usr/local/bin/odroid-utility.sh https://raw.githubusercontent.com/mdrjr/odroid-utility/master/odroid-utility.sh
chmod +x /usr/local/bin/odroid-utility.sh
source ~/.bashrc

printf "\nInstalling ncurses library\n\n"
apt-get --yes install libncurses5-dev libncursesw5-dev

printf "\nGiving executable permissions to installation scripts"
chmod +x *.sh

printf "\nInstalling ncurses library\n\n"
apt-get --yes install libncurses5-dev libncursesw5-dev

printf "\nInstalling Python\n\n"
apt-get --yes install python-dev python-pip python3-dev python3-pip

## NOT WORKING ##
# printf "\nConfiguring WiFi dongle with Realtek 8821cu driver"
# cd wifisetup
# make
# sudo make install

















