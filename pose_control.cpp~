#include "ros/ros.h"
#include "geometry_msgs/Pose.msg" 
#include "<ee_cart_imped_action/ee_cart_imped_arm.hh>"
#include "<ee_cart_imped_msgs/EECartImpedGoal.h>"
#include "<kdl/frames.hpp>" 

using namespace KDL; 

/**
* Receive pose and orientation information from ROS topics and send it on to ee_cart for
* simulator control.
*/

/* Contains the 10 poses received so far */ 
geometry_msgs::Pose poseBuffer[10]; 

/* Pointer into the pose buffer to see if it is full */
int pointer; 

/* KDL rotation from haptics frame to robot frame */
KDL::Rotation rotation = Rotation(0, 0, -1, -1, 0, 0, 0, 1, 0); 

void poseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
  pointer++; 
  if (pointer == 9) {
    ee_cart_imped_msgs::EECartImpedGoal traj; 
    for (int i = 0; i < 10; i++) {
      EECartImpedArm::addTrajectoryPoint(traj, 0.5, 0, 0, 0, 0, 0, 1,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 4, "/torso_lift_link");
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poseListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, poseCallback);

  ros::spin();

  return 0;
}
