#!/bin/bash
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse

#Set up computer accept software from packages.ros.org 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Set up your key 
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update 
#Install ROS Noetic Desktop Full and Gazebo11
sudo apt install ros-noetic-desktop-full

#source to every terminal
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc