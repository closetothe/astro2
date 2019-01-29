printf "\n\n--- THIS IS GOING TO TAKE A WHILE ---\n\n"

printf "\n\e[34mInstalling librealsense dependencies\e[0m\n"

sudo apt-get -y install libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev python3-dev python-dev

printf "\n\e[34mDownloading librealsense 2...\e[0m\n"

#git clone --single-branch --branch odroid_build https://github.com/freemanlo/librealsense.git ~/Desktop/librealsense

if [ -d ~/Desktop/librealsense2 ]
then
	rm -rf ~/Desktop/librealsense2
fi

git clone https://github.com/IntelRealSense/librealsense.git ~/Desktop/librealsense2

printf "\n\e[34mInstalling librealsense 2...\e[0m\n"

cd ~/Desktop/librealsense2
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger 
sudo modprobe uvcvideo

#sudo ./scripts/patch-realsense-ubuntu-odroid-xu4-4.14.sh
sudo ./scripts/patch-realsense-ubuntu-odroid.sh

mkdir ~/Desktop/librealsense2/build
cd ~/Desktop/librealsense2/build

cmake .. -DBUILD_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=Release

#cmake ../ -DCMAKE_BUILD_TYPE=Release

make uninstall && make clean && make -j4 && sudo make install && printf "\n\e[32mSuccessfully installed RealSense SDK 2\e[0m\n\n"


