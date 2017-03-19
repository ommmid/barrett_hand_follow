This package uses actionlib to send commands to JointState topic to control Barret Hand joints.

The server (bh_follow_server.cpp) receives commands from a client (here is another cpp file node, bh_follow_server.cpp but could be MoveIt or anyother client) through actionlib Goal to topic /bhand_node/command. 

This packages is written to do a one point trajectory but it can be easily extended to do more than one point trajectory by 
