# Monocular SLAM for UAVs in Forest Environment

This project was developed troughout the spring semester of 2021, at Aalborg University. The purpose of this project was to implement monoSLAM into a small UAV (Parrot AR drone) with the purpose of performing Search and Rescue operations in a forest environment. The project is supported on the use of the ORB-SLAM2 library for ROS and the Tum Simulator for the drone.

This project was developed and tested for Ubuntu 18.04 + ROS Melodic.

## Dependencies

In order to run this project, you will need to follow the following dependencies:

[ORB-SLAM2 (ros)](http://wiki.ros.org/orb_slam2_ros)<br/>
[Tum SImulator (melodic)](https://github.com/surajmahangade/tum_simulator_melodic)<br/>
[ardrone_joystick](https://github.com/acpopescu/ardrone_joystick) - just for testing purpose, not mandatory<br/>

Please follow all the instructions on the above websites on how to install these dependencies

## Installation

Start by creating a [catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) worspace:

```bash
mkdir -p ~/catkin_ws/src
```
Install all the dependencies in the /src folder of the workspace. After this, clone this repository:

```bash
git clone https://github.com/arturfabricio/control
```
You can now make your catkin workspace:

```bash
cd ~/catkin_ws/
catkin_make
```
Everything should be properly installed now. Don't forget to source.

```bash
source devel/setup.bash
```

## Usage

We currently only have a test .launch file which starts the simulation, ORB-SLAM2, rviz and joystick control. Test by typing the following on a terminal:

```bash
roslaunch control test.launch
```

## Contributing

## License

## Videos
