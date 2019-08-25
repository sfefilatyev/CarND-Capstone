
Updated at 1:15PM on Saturday.  Fresh updates will be made in Google Docs and ported over here in batches for efficiency (https://docs.google.com/document/d/10jzkAwM7JccJiGRzyhoEpzobwL02YP0_Dq7CHTU0c38/edit#)

### Team Members
1. Sergiy Feyfilatyev (Github: )
2. Dmitriy Litvak (Github: )
3. Devdatta Gangal (Github: )
4. Reheman Baikejiang (Github: )

### Goal
The rubrik of the Capstone project is quite straightforward - did the car navigate the track successfully? The submitted code must work successfully to navigate Carla around the test track.

### Results
1. Carla drives successfully with our code
Video Insert
2. Runs successfully on the Highway (Simulator)
Video insert
3. Runs successfully on the Test Lot (Simulator)
Video Insert



### Installation instructions


* Port Forwarding
We looked up the instructions from [the course (3. Getting Started) here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Port+Forwarding.pdf) for port forwarding

* Running the code in simulator

1. Clone the project repository
```bash
git clone https://github.com/sfefilatyev/CarND-Capstone/
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

* Real world testing (will be updated)
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Architecture

<img src="final-project-ros-graph-v2.png" alt="Architecture" width="490" height="338">

We follow the architecture as prescribed in the class instructions. The project runs with [ROS](http://www.ros.org/) and is divided into the following modules :
 - `tl_detector` uses the camera to detect the traffic lights' color
 - `twist_controller` handles the control of the car
 - `waypoint_follower` makes sure the car follow the trajectory
 - `waypoint_loader` loads the route the car is going to follow
 - `waypoint_updater` adapts the car's route to the situation (eg. traffic light)

### Insights on what we built
* Traffic Light detector
* Twist Controller
* Waypoint Follower
* Waypoint Loader
* Waypoint Updater
