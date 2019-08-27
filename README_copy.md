
Updated at 10:45PM on Monday.  Fresh updates will be made in Google Docs and ported over here in batches for efficiency (https://docs.google.com/document/d/10jzkAwM7JccJiGRzyhoEpzobwL02YP0_Dq7CHTU0c38/edit#).

### Team Members
1. Sergiy Feyfilatyev (Github: sfefilatyev)
2. Dmitriy Litvak (Github: dlitvak)
3. Devdatta Gangal (Github: devdatta-work)
4. Reheman Baikejiang (Github: bakijan)
5. Devunuri Sai Praneeth (Github: )

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
* Native Installation
1. We use the Udacity provided virtual machine with ROS and Dataspeed DBW already installed - settings at 2CPU, 2GB system memory and 25GB free space
2. Simulator Downloaded the [Udacity Simulator[(https://github.com/udacity/CarND-Capstone/releases)] on the client machine.  It works best in the "simple" version at 640 x 480


* Port Forwarding
We looked up the instructions from [the course (3. Getting Started) here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Port+Forwarding.pdf) for port forwarding

* Changes to the Requirements.txt
While debugging camera topic we stumbled on a the following bug:
https://github.com/udacity/CarND-Capstone/issues/147 We updated our requirements.txt with Pillow pointing to version 4.3 to address dependencies.

* Real world testing (will be updated)
See below

* Suggestions
I wish Virtual Workspace had internet access so that instead of uploading files it would be possible to clone a repo.

### Issues we ran into
1. Requirements.txt was modified to point pillow to version 4.3 (see above).  We had an issue with cv.Bridge that was traced back to https://github.com/udacity/CarND-Capstone/issues/147.  This required upgrade to pillow version 4.3
- We do want to call out that while you mention not to change the requirements.txt, you have used different kinds of settings yourself in the Virtual Workspace
- This has caused a lot of confusion and wasted time
2. Need to mention obstacles were envisioned in the beginning with some stub code provided, but never actually finished in the project.
3. We ran into issues with the car’s deceleration.  We observed crazy jerk after detection of the stop-light.  Seeking inspiration from  https://github.com/justinlee007/CarND-Capstone we adjusted the waypoints to ensure that the velocity was under the maximum possible velocity as given by v2=2aS where a = deceleration and S = distance until the stop-line.
- The maximum deceleration chosen was 0.5m/s2 initially.
- The issue still persisted because the car now had a discrete drop in velocity for the first waypoint where correction needs to happen
- We worked on many different solutions only to realize later that this behavior is consistent with physics.  
- The first thing we changed was to tolerate higher deceleration (now changed to 5m/s2 which is 0.5G significantly less than 2G which is considered hard braking in industry)
- Second thing was to increase the lookahead from 50 waypoints to 100
4. We also stumbled upon the “SteeringReport” issue (https://knowledge.udacity.com/questions/46645). The current install of DBW breaks because the steering_wheel_angle_cmd d field, which is populated in bridge.py no longer exists, and we had to change it accordingly inside styx node → the bridge.py (line number 102). The issue is described https://github.com/udacity/CarND-Capstone/issues/306


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


### Other instructions
* Real world testing instructions
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
