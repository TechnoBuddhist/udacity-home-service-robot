# Udacity Final Project - "Home Service Robot"
My final project for the Udacity Robotics Software Engineer Nanodegree 2019.

## Getting Started
You need [ROS](http://ros.org). If you aren't familiar with [ROS](http://ros.org) then this project is probably no good to you whatsoever!

I have used ROS Kinetic as this is used on the nanodegree.

I'm not going to talk you through how to use ROS, there are plenty of tutorials online and the [ROS Wiki](http://wiki.ros.org) is particularly useful. You will need a ROS workspace, either one you have currently or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

You will also need [RVIZ](http://wiki.ros.org/rviz/UserGuide) and [Gazebo](http://gazebosim.org/).

The project is launched using a shell script tht launches several xterm terminals for each of several different Ros packages.

### Prerequisites
- An installed ROS system with rviz, gazebo.
- xterm installed `sudo apt-get install xterm`
- A ROS workspace

### Installing
You should [create a new package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

Copy/clone this project into your workspace/src directory.

Since there are a few package dependencies I have provided an install_pkgs.sh file that will clone the required Ros packages from github into the workspace/src directory.

```
cd <your workspace>/src
sh scripts/install_pkgs.sh
```

## Usage
This project will create an rtabmap database for viewing created map and robot path. You may want to change the location of this database by either passing the parameter `database_path` to the mapping.launch command or change the default in the mapping.launch file(src/basketbot/launch/mapping.launch). By default the path is `~/.ros/rtabmap.db`.

You will need 3 terminals running.

**Terminal 1**
In your ROS workspace run the world with basketbot(launches rviz and gazebo):-

```
catkin_make
source devel/setup.bash
roslaunch basketbot world.launch
```

**Terminal 2**
In your ROS workspace run the teleop module to control the baskebot :-

```
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```


**Terminal 3**
In your ROS workspace launch the rtabmap module to perform RTAB :-

```
source devel/setup.bash
roslaunch basketbot mapping.launch 
```

## Viewing RTAB Created Map ##
###My Generated Map
My generated DB is too big for github so is stored on Google Drive in both zipped and unzipped format. Simply download and issue the same command as below.

[unzipped rtabmap.db](https://drive.google.com/open?id=1IF21DfZuhObm67K9NsVUwdalZ1X2t0e4)

[zipped rtabmap.db.zip](https://drive.google.com/open?id=1VCui1ZcR0mGmwyNhvYoG2pp9KL-ROlm9)

###Your Own Generated Map
When you have fininshed navigating the environment, you can view the created map and robot path by viewing the map in rtabmap-databaseViewer. In a terminal :-

```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

## Future Work
Work out how/why localization.launch is not doing anything.

## Contributing
No contributions accepted this is **my project** for the course.

## License
GNU GPLv3

## Attributions & Acknowlegements
