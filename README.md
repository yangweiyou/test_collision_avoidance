test_collision_avoidance
===
This test module is based on **Centauro** robot, so remember to set a proper xbot config file. Below are the steps to run this test module.

First, turn on roscore and xbotcore individually.

Terminal 1:

	$ roscore
Terminal2:

	$ XBotCore -D
	
Then turn on rviz.

Terminal 3:

	$ rosrun rviz rviz -d test_collision_avoidance.rviz

At last, turn on XBotCommunicationPlugin and run the test code:

Terminal 4:

	$ rosservice call /XBotCommunicationPlugin_switch 1
	$ rosrun test_collision_avoidance test_collision_avoidance
	
After doing all thest, you should be able to see something like the enclosed video.