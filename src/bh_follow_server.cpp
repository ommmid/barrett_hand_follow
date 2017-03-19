#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class BHTrajectoryFollower
{
protected:
  ros::NodeHandle nh_;

  //Actionlib
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

  std::string action_name_;

  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;

  // messages:
  ros::Publisher pub_;

public:

  BHTrajectoryFollower(std::string name) :  
    as_(nh_, name, boost::bind(&BHTrajectoryFollower::executeCB, this, _1), false),
    action_name_(name)
  {

    pub_ = nh_.advertise<sensor_msgs::JointState>("bhand_node/command", 10);

    as_.start();

  }

  ~BHTrajectoryFollower(void)
   {
   }


  // callback function
void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal){

   // make sure that the action has not been cancelled
  if (!as_.isActive())
    return; 

  // helper variables
      ros::Rate r(1);
      bool success = true;

  sensor_msgs::JointState msg;

    msg.header.seq = goal->trajectory.header.seq;
    msg.header.stamp.sec = goal->trajectory.header.stamp.sec;
    msg.header.stamp.nsec = goal->trajectory.header.stamp.nsec;
    msg.header.frame_id = goal->trajectory.header.frame_id;

    // I am trying for one point and all the joints
    for (int i=0; i < 4; i++) {

  // trajectory_msgs::JointTrajectory => trajectory
  msg.name.push_back(goal->trajectory.joint_names[i]);
  msg.position.push_back(goal->trajectory.points[0].positions[i]);
  msg.velocity.push_back(goal->trajectory.points[0].velocities[i]);
  msg.velocity.push_back(goal->trajectory.points[0].effort[i]);

     // publish the velocity command
    pub_.publish(msg);

    r.sleep();
}
    if(success)
        {

          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          as_.setSucceeded(result_);
        }

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bh_server");
  BHTrajectoryFollower bh_server("bh_server");

  ros::spin();

return 0;
}
