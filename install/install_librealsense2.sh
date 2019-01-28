printf "\n\n--- THIS IS GOING TO TAKE A WHILE ---\n\n"

printf "\nInstalling librealsense dependencies\n"

sudo apt-get -y install libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev python3-dev python-dev

printf "\nDownloading librealsense...\n"

#git clone --single-branch --branch odroid_build https://github.com/freemanlo/librealsense.git ~/Desktop/librealsense

git clone https://github.com/IntelRealSense/librealsense.git ~/Desktop/librealsense

printf "\nInstalling librealsense...\n"

cd ~/Desktop/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger 
sudo modprobe uvcvideo

#sudo ./scripts/patch-realsense-ubuntu-odroid-xu4-4.14.sh
sudo ./scripts/patch-realsense-ubuntu-odroid.sh

mkdir ~/Desktop/librealsense/build
cd ~/Desktop/librealsense/build

cmake .. -DBUILD_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=Release

#cmake ../ -DCMAKE_BUILD_TYPE=Release

sudo make uninstall && sudo make clean && sudo make -j4 && sudo make install


