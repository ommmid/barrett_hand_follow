This package uses the actionlib to send commands to JointState topic to control Barret Hand joints.

The server (bh_follow_server.cpp) receives commands from a client (here is another cpp file node, bh_follow_server.cpp but could be MoveIt) through actionlib Goal to topic /bhand_node/command. 
