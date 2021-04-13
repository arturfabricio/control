# Monocular SLAM for UAVs in Forest Environment

This project was developed troughout the spring semester of 2021, at Aalborg University. The purpose of this project was to implement monoSLAM into a small UAV (Parrot AR drone) with the purpose of performing Search and Rescue operations in a forest environment. The project is supported on the use of the ORB-SLAM2 library for ROS and the Tum Simulator for the drone.

This project was developed and tested for Ubuntu 18.04 + ROS Melodic.

## Dependencies

In order to run this project, you will need to follow the following dependencies:

[ORB-SLAM2 (ros)](http://wiki.ros.org/orb_slam2_ros)
[Tum SImulator (melodic)](https://github.com/surajmahangade/tum_simulator_melodic)
[ardrone_joystick](https://github.com/acpopescu/ardrone_joystick) - just for testing purpose, not mandatory

Please follow all the instructions on the above websites on how to install these dependencies

## Installation

Start by creating a [catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) worspace:

´´´bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
´´´

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install foobar.

```bash
pip install foobar
```

## Usage

```python
import foobar

foobar.pluralize('word') # returns 'words'
foobar.pluralize('goose') # returns 'geese'
foobar.singularize('phenomena') # returns 'phenomenon'
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
