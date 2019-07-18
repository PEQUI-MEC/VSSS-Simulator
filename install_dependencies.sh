echo 'Installing dependencies'
sudo apt install libglfw3 libglfw3-dev libglu1-mesa-dev freeglut3-dev -y

echo "Installing ROS"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

echo "Installing vsss_msgs"
cd vsss_msgs_ws
VSSS_MSGS_LOCATION=/home/$USER/.vsss/vsss_msgs
catkin_make install -DCMAKE_INSTALL_PREFIX=$VSSS_MSGS_LOCATION
echo 'source /home/$USER/.vsss/vsss_msgs/setup.bash' >> ~/.bashrc
source ~/.bashrc
