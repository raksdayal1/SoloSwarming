1. Start the Ground station Ros node
	$ rosrun swarm_control GroundControlStation.py

2. In another terminal launch the swarm_launch file
	$ roslaunch swarm_control swarm_launch.launch

3. Once the swarm_launch is complete it will output a message to launch the swarm controller. In another terminal run 
	$ roslaunch swarm_control swarm_control.launch