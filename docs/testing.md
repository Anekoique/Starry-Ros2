##### configure env

```shell
source install/setup.sh
printenv | grep -i ROS
```

##### ros2node

A node is a fundamental ROS 2 element that serves a single, modular purpose in a robotics system.

```shell
ros2 node list
ros2 node info <node_name>
```

##### ros2topic

Nodes publish information over topics, which allows any number of other nodes to subscribe to and access that information. 

```shell
ros2 topic list

ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
ros2 topic echo /hello
ros2 topic pub /hello std_msgs/msg/String "{data: 'hi from cli'}" -r 5

ros2 topic echo <topic_name> --once
ros2 topic info <topic_name>

ros2 topic hz /chatter
ros2 topic bw /chatter
ros2 topic find /chatter<topic_type>
```

##### ros2service

A service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.

```shell
ros2 service list

ros2 run demo_nodes_cpp add_two_ints_server &
ros2 run demo_nodes_cpp add_two_ints_client
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 35}"

ros2 service type /add_two_ints
ros2 service info /add_two_ints
ros2 service find example_interfaces/srv/AddTwoInts
```

##### ros2launch

Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

```shell
ros2 launch demo_nodes_cpp introspect_services_launch.py
```

##### ros2param [leave problem]

Nodes have parameters to define their default configuration values. 

```shell
ros2 param list
```

##### ros2action [leave problem]

Actions are like services that allow you to execute long running tasks, provide regular feedback, and are cancelable.

```shell
ros2 action list
ros2 action type <action_name>
ros2 action info <action_name>
```

##### ros2doctor

`ros2doctor` will inform you of problems in your ROS 2 setup and running systems. You can get a deeper look at information behind those warnings by using the `--report` argument.

```shell
ros2 doctor
ros2 doctor --report
```

