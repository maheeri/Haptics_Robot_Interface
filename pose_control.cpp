#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>
#include <kdl/frames.hpp>

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

/* Coordinates for the current robot Quaternion transform */
double ox;
double oy;
double oz;
double ow; 

void poseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
  pointer++; 
  poseBuffer[pointer] = *pose; 

  if (pointer == 9) {
    EECartImpedArm arm("r_arm_cart_imped_controller");
    ee_cart_imped_msgs::EECartImpedGoal traj; 
    for (int i = 0; i < 10; i++) {

      geometry_msgs::Pose currPose = poseBuffer[i];
      geometry_msgs::Quaternion qtn = currPose.orientation;
      geometry_msgs::Point pnt = currPose.position; 

      /* Rotation given by the haptic quaternion */
      KDL::Rotation qRotation = KDL::Rotation::Quaternion(qtn.x, qtn.y, qtn.z, qtn.w);

      /* Transforming from haptic frame to robot frame */
      KDL::Rotation finRotation = rotation * qRotation; 

      /* Vector from the position of the haptic point */
      KDL::Vector pVector = Vector(pnt.x, pnt.y, pnt.z); 

      /* Transforming position from haptic frame to robot frame */
      KDL::Vector finVector = rotation * pVector; 

      /* Store the current quaternion of the rotation */
      finRotation.GetQuaternion (ox, oy, oz, ow); 

      EECartImpedArm::addTrajectoryPoint(traj, finVector.x(), finVector.y(), finVector.z(),
				     ox, oy, oz, ow,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 4, "/torso_lift_link");
    } 
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poseListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("hapticsPose", 1000, poseCallback);

  ros::spin();

  return 0;
}
