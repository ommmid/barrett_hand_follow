This package uses actionlib to send commands to JointState topic to control Barret Hand joints.

The server (bh_follow_server.cpp) receives commands from a client (here is another cpp file node, bh_follow_server.cpp but could be MoveIt or anyother client) through actionlib Goal to topic /bhand_node/command. 

In this packages, the clinet is sending one point trajectory ind=0 which can be easily extended to more points by editing ind in bh_follow_client.cpp.

It is noteworthy that package https://github.com/jontromanab/barrett_hand_follow has already been created for the same purpose however the current package is written by c++ besides providing the client node for more convenience.
