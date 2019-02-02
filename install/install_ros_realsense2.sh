printf "\n\e[34mInstalling ros-realsense v2.1.4 from source\e[0m\n\n"

if [ ! -d ~/ws/src ]
then 
	printf "\n\e[31mCatkin workspace ~/ws not found.\e[0m\n\n"
else
	git clone https://github.com/intel-ros/realsense.git ~/Downloads/realsense
	# Switch to stable version 2.1.4
	# (Better would be the newest version, but I'm
	#  not sure how to do that programmatically)
	cd ~/Downloads/realsense
	git checkout tags/2.1.4 -b astro_dev
	cd ~/Downloads
	mv realsense ~/ws/src

	cd ~/ws
	catkin_make clean
	catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
	catkin_make install

	printf "\n\e[32mros-realsense v2.1.4 installed to ~/ws/src\e[0m\n\n"

	printf "\n\e[34mInstalling rgbd-launch\e[0m\n\n"
	sudo apt-get install ros-kinetic-rgbd-launch
	printf "\n\e[32mDone installing realsense ros wrapper\e[0m\n\n"
fi
