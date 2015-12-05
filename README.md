# COROS

Project Repo for Turtlebot Project for ECE6562.

## Setup

Follow the below steps to get up and running with everything.

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


## Running

To run this package with ROS, issue the below command from anywhere.

```
roslaunch turtlebot_gt turtlebot.launch
```

## Adding Nodes

Add a new nodes, and remap current topics by editing [`turtlebot.launch`](./src/turtlebot_gt/launch/turtlebot.launch).
