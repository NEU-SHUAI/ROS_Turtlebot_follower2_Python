# COROS

Project Repo for Turtlebot Project for ECE6562.

## Setup

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

### Python

Since ROS does not handle 


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


## Adding Nodes

Add new nodes, and remap current topics by editing [`turtlebot.launch`](./src/turtlebot_gt/launch/turtlebot.launch).
