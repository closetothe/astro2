## Installation instructions

1. Download arduino for Linux. This should be a folder named "arduino-[version]".
2. Rename it to "arduino".
3. Put it in this directory.
4. Run `sudo chmod +x install_everything.sh`
5. Run `sudo ./install_everything.sh`

Note that `install_ros_deps.sh` will create a catkin workspace at `~/ws`. If this doesn't work for some reason, create your own with the name `ws` and run `install_ros_differential_drive.sh` to install the (depreciated) `differential_drive` dependency.
