#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "bh_client");

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("bh_server", true);

  ROS_INFO("===> Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time


  ROS_INFO("===> Action server started, sending goal.");
  // Here a goal message is created, the goal value is set and sent to the action server.
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp.sec = 0;
  goal.trajectory.header.stamp.nsec = 0;
  goal.trajectory.header.frame_id = "";

  goal.trajectory.joint_names.push_back("bh_j11_joint");
  goal.trajectory.joint_names.push_back("bh_j12_joint");
  goal.trajectory.joint_names.push_back("bh_j22_joint");
  goal.trajectory.joint_names.push_back("bh_j32_joint");

     goal.trajectory.points.resize(1);

     // trajectory point. for each point we need to give a value to the joints (as their positions)
     // Positions
     int ind = 0;
     goal.trajectory.points[ind].positions.resize(4);
     goal.trajectory.points[ind].positions[0] = 0.3;
     goal.trajectory.points[ind].positions[1] = 0.2;
     goal.trajectory.points[ind].positions[2] = 0.1;
     goal.trajectory.points[ind].positions[3] = 1.2;


     // Velocities
     goal.trajectory.points[ind].velocities.resize(4);
     goal.trajectory.points[ind].effort.resize(4);
     for (size_t j = 0; j < 4; ++j)
     {
       goal.trajectory.points[ind].velocities[j]=0.0;
       goal.trajectory.points[ind].effort[j] = 0.0;
     }
     // To be reached 1 second after starting along the trajectory
     goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    ac.sendGoal(goal);

    ROS_INFO("===> the goal is sent.");

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
