## ASTRO 2 Installation Instructions
##### (Requires Ubuntu 16.04)

The ASTRO 2 robot uses both an ODROID XU4 (armhf) and a MacBook Pro running Ubuntu 16.04 (amd64). Installation scripts that work in both environments are in this folder, and the rest are separated in their respective folders.


NOTE: `install_everything` is a bit finicky (it won't stop if one of the scripts fails). I suggest running the scripts individually in roughly this order:

1. Core
2. Arduino
3. OpenCV, OpenGL, and CMake 5
4. RealSense SDK 2.0
5. ROS  
- **IMPORTANT**: `source ~/.bashrc`
6. ROS Dependencies 
- Installs the ROS Arduino library at the default `~/Arduino/libraries`
- Creates a catkin workspace at `~/ws`
- Remember to `source ~/.bashrc`
7. ROS Diff Drive
8. ROS RealSense


### ODROID XU4

1. Download arduino for Linux from the Arduino website. This should be a folder named "arduino-[version]".
2. Rename it to "arduino".
3. Put it in the `odroid` directory.
4. Open a terminal and `cd` into the `odroid` directory.
5. Run `sudo chmod +x install_everything.sh`
6. Run `sudo ./install_everything.sh`

### Intel/AMD64

1. Download arduino for Linux from the Arduino website. This should be a folder named "arduino-[version]".
2. Rename it to "arduino".
3. Put it in the `amd64` directory.
4. Open a terminal and `cd` into the `amd64` directory.
5. Run `sudo chmod +x install_everything.sh`
6. Run `sudo ./install_everything.sh`
