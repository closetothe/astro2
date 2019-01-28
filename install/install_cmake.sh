# CMake 3.8+ is not available through apt
# and is 'required' by librealsense

# This script builds the newest CMake from source

printf "\n\e[34mInstalling the newest CMake\e[0m\n\n"

if [ -d ~/Downloads/cmake_source ]1m
then
	rm -rf ~/Downloads/cmake_source
fi

git clone https://github.com/Kitware/CMake.git ~/Downloads/cmake_source

cd cmake_source
./bootstrap && make && sudo make install && printf "\n\e[32mSuccessfully installed CMake\e[0m\n\n"

