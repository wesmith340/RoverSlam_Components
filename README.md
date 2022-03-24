# Steps to compile

### install noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop # for rviz tools
sudo apt install ros-noetic-ros-base # for base with no gui components
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

### install freenect drivers
sudo apt-get update
sudo apt-get install cmake build-essential libusb-1.0-0-dev
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build && cd build
cmake -L ..
make
sudo make install

### create workspace
sudo apt install python3-catkin-tools python3-osrf-pycommon
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/

### install freenect_stack and dependencies
cd src
git clone https://github.com/ros-drivers/freenect_stack.git
git clone https://github.com/ros-perception/image_common.git
git clone https://github.com/ros-drivers/rgbd_launch.git
git clone https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/ros-perception/image_pipeline.git
git clone https://github.com/ros/geometry2.git

cd ..

### time to build
rosdep install --from-paths src --ignore-src -r -y
catkin build

### install rtabmap
sudo apt-get install ros-noetic-rtabmap-ros

# It's showtime

### Terminal 1
source devel/setup.bash
roslaunch freenect_launch freenect.launch depth_registration:=true

### Terminal 2
roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start --Vis/MaxFeatures 500 --Mem/ImagePreDecimation 2 --Mem/ImagePostDecimation 2 --Kp/DetectorStrategy 6 --OdomF2M/MaxSize 1000 --Odom/ImageDecimation 2" rtabmapviz:=false
