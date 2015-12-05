# COROS

Project Repo for Turtlebot Project for ECE6562.

## Setup

Follow the below steps to get up and running with everything.

1. Make sure you have ROS installed and setup already.
2. Clone this repository
```
git clone https://github.gatech.edu/rramachandran3/COROS
```
3. Go to folder that was created with the above command
```
cd COROS
```
4. Run the setup script with regular privileges - accepting the script's changes to `~/.bashrc` is highly recommended
```
./setup.sh
```
5. Source your `~/.bashrc` file again - that's it!
```
. ~/.bashrc
```

## Running

To run this package with ROS, issue the below command from anywhere.

```
roslaunch turtlebot_gt turtlebot.launch
```
