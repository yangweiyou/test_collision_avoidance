pholus_docker
===
Locomotion modules (legged and wheeled) as well as basic motion control modules are developed.

### 1. Non real-time plugin
There are four non real-time plugins:

- control_interface: an interface integrating four functionalities: torso regulation, driving, walking and arm testing.

- search_control: a wrapper for torso_control to communicate with perception module

- torso_control: regulate the torso pose in addition to driving

- env_cast: get the pose of objects in simulation

Command to run:

	$ rosservice call /XBotCommunicationPlugin_switch 1
	$ rosrun pholus_locomotor _pluginName_
	
To make use of the control_interface plugin, a standard joystick ros message (sensor_msgs::Joy) can be sent.
Its definition (http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Joy.html) is listed below:

	Header header   # timestamp in the header is the time the data is received from the joystick
	float32[] axes  # the axes measurements from a joystick
	int32[] buttons # the buttons measurements from a joystick 

#### Task mode
Four task modes can be chosen by setting on specific buttons:

|Mode			|Button			|Value	|
|---					|---				|---		|
|Torso regulation task	|	buttons[4]	|0/1		|
|Driving task			|	buttons[5]	|0/1		|
|Walking task			|	buttons[6]	|0/1		|
|Arm grasping task		|	buttons[7]	|0/1		|

The task mode only needs to set once before switching to another one. In each mode, there are several other functional buttons and axes which can used to interact with the robot.

#### Torso regulation task

|Function		|Axe/Button	|Range	|
|---			|---			|---		|
|Pitch angle	|axes[1]		|-1~1	|
|Roll angle	|axes[0]		|-1~1	|
|Yaw angle	|axes[2]		|-1~1	|


#### Driving task

|Function				|Axe/Button	|Range	|
|---					|---			|---		|
|Forward velocity		|axes[1]		|-1~1	|
|Lateral velocity		|axes[0]		|-1~1	|
|Turing				|axes[2]		|-1~1	|
|Turn on auto-docker	|buttons[1]	|0/1		|
|Turn off auto-docker	|buttons[2]	|0/1		|


#### Walking task

|Function			|Axe/Button	|Range	|
|---				|---			|---		|
|Forward velocity	|axes[1]		|-1~1	|
|Lateral velocity	|axes[0]		|-1~1	|
|Turing			|axes[2]		|-1~1	|
|Start walking		|buttons[1]	|0/1		|
|Stop walking		|buttons[2]	|0/1		|


#### Arm grasping task

|Function		|Axe/Button	|Range	|
|---			|---			|---		|
|Fetch		|buttons[1]	|0/1		|
|Retrieve		|buttons[2]	|0/1		|

### 2. Real-time plugin
Only one real-time plugin was implemented by now:

- real_time_locomotor: real time version of locomotion module

Command to run:

	$ rosservice call /real_time_locomotor_switch 1