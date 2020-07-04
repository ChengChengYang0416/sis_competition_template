# send_command_to_robot
service server and client for send command to robot

## Usage
1. Git clone and build this project
	```
	$ git clone https://github.com/ChengChengYang1997/send_command_to_robot.git
	$ cd <your workspace>
	$ catkin_make
	```

2. roscore and run the server and client
	```
	roscore
	rosrun send_command_to_robot robot_command_server
	rosrun send_command_to_robot robot_command_client
	```
