1. Building the ros package
 1.1. create a rosworkspace as follows e.q.
	$ mkdir ~/ros_ws && cd ~/ros_ws
	$ mkdir src
	$ catkin_make
 1.2. cd into the 'src' directory, and create a directory and             clone the source code into the directory e.q.
	$ cd src
	$ mkdir swarm_control && cd swarm_control
	$ git clone https://github.com/rakshit1991/Swarming.git
 1.3. Make the ros package from the base ros workspace
	$ cd ~/ros_ws
	$ catkin_make

2. Running the ros package
 2.1. Start the Ground station Ros node
	$ rosrun swarm_control GroundControlStation.py

 2.2. In another terminal launch the swarm_launch file
	$ roslaunch swarm_control swarm_launch.launch

 2.3. Once the swarm_launch is complete it will output a message to launch the swarm controller. In another terminal run 
	$ roslaunch swarm_control swarm_control.launch