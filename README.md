# COROS

Project Repo for Turtlebot Project for ECE6562.

## ROS Setup

These sections give a basic rundown of the steps that should be taken to setup a computer for testing software, or testing code on the TurtleBot itself.

### Catkin Setup

Follow these steps to setup this repo with ROS. You will be able to reference the nodes/topics in this repo once complete.

0. Make sure you have ROS installed and setup already.
0. Clone this repository

	```
	git clone https://github.gatech.edu/rramachandran3/COROS .
	```

0. Go to folder that was created with the above command

	```
	cd ./COROS
	```

0. Run the [setup script](./setup.sh) with regular privileges - accepting the script's changes to `~/.bashrc` is highly recommended

	```
	./setup.sh
	```

0. Source your `~/.bashrc` file again - that's it!

	```
	. ~/.bashrc
	```

### Python Libraries

To install python packages that aren't supported natively in ROS, use `pip`.

```
pip install -r requirements.txt
```

### Repository Lists

To install the required ROS repository references, run the [`install-meta`](./install-meta) script. Be sure to edit the first few lines of the script for installing for correct distro of ROS.

```
./install-meta
```


## Running

To run this package with ROS, issue the below command from any directory.

```
roslaunch turtlebot_gt turtlebot.launch
```

To open a graph of all nodes/topics and their conenctions, set the `debug` argument when launching.

```
roslaunch turtlebot_gt turtlebot.launch debug:=true
```

If you wish to test something outside the lab using your own camera, specify the `no_kinect` argument.

```
roslaunch turtlebot_gt turtlebot.launch no_kinect:=true
```

To update the tracking profile values, invoke the [`profiler.launch`](./src/turtlebot_gt/launch/profiler.launch) file instead. Once started, a window will appear. Place the ball within the circle, then press `q` to store the new profile.

```
roslaunch turtlebot_gt profiler.launch
```

## Adding Nodes

Add new nodes, and remap current topics by editing [`turtlebot.launch`](./src/turtlebot_gt/launch/turtlebot.launch).
