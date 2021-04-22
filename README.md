# Monocular SLAM for UAVs in Forest Environment

This project was developed troughout the spring semester of 2021, at Aalborg University. The purpose of this project was to implement monoSLAM into a small UAV (Parrot AR drone) with the purpose of performing Search and Rescue operations in a forest environment. The project is supported on the use of the ORB-SLAM2 library for ROS and the Tum Simulator for the drone.

This project was developed and tested for Ubuntu 18.04 + ROS Melodic + Gazebo 9.

## Dependencies

In order to run this project, you will need to install the following dependencies:

[ORB-SLAM2 (ros)](http://wiki.ros.org/orb_slam2_ros)<br/>
[Tum Simulator (melodic)](https://github.com/surajmahangade/tum_simulator_melodic)<br/>
[ardrone_joystick](https://github.com/acpopescu/ardrone_joystick) - just for testing purpose, not mandatory<br/>
[octomap](http://wiki.ros.org/octomap)<br/>
[octomap_mapping](http://wiki.ros.org/octomap_mapping)<br/>
[Hector-gazebo packages](https://answers.ros.org/question/281462/drone-keeps-rising-in-simulation-after-takeoff/) <br/>

Please follow all the instructions on the above websites on how to install these dependencies

## Installation

##### Install the Desktop-Full version of [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

##### Upgrade gazebo to the newest version 9.XX:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo9
sudo apt upgrade libignition-math2
```

##### Create a [workspace folder](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in the home directory

```bash
mkdir -p ~/catkin_ws/src
```

### Installing dependencies:<br/>

##### Aptitude packages:

```bash
sudo apt install libeigen3-dev libsdl1.2-dev ros-melodic-octomap ros-melodic-hector-gazebo ros-melodic-hector-sensors-gazebo ros-melodic-xacro-tools
```

##### Clone the repositories into the src directory of your catkin workspace:

```bash
git clone https://github.com/surajmahangade/tum_simulator_melodic.git && git clone https://github.com/dsapandora/ardrone_autonomy.git && git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git && git clone https://github.com/OctoMap/octomap_mapping.git
```

### Installing the package:<br/>

##### Clone the package into the src directory of your catkin workspace

```bash
git clone https://github.com/arturfabricio/control
```

##### You can now build your catkin workspace:

```bash
cd ~/catkin_ws/
catkin_make
```

##### Everything should be properly installed now. Don't forget to source.

```bash
source devel/setup.bash
```

## Usage

We currently only have a test .launch file which starts the simulation, ORB-SLAM2, rviz and joystick control. Test by typing the following on a terminal:

```bash
roslaunch control test.launch
```

If you do not wish to use joystick control run:

```bash
roslaunch control test_nojoy.launch
```

## Troubleshooting

### Octomap catkin_make error:<br/>

Could not find a package configuration file provided by "octomap_ros" with
any of the following names:

octomap_rosConfig.cmake
octomap_ros-config.cmake

This happens because of missing octomap_ros libraries, and can be fix by installing the following:

```bash
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-octomap-server
```

## Contributing

## License

## Videos
