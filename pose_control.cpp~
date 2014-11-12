#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>

#define TGT_FRAME "/l_gripper_tool_frame"
#define TRAJ_RATE 0.05 

using namespace KDL; 

/**
* Receive pose and orientation information from ROS topics and send it on to ee_cart for
* simulator control.
*/ 

/* KDL rotation from haptics frame to robot frame (column-major) */
KDL::Rotation rotation = Rotation(0, 0, -1, -1, 0, 0, 0, 1, 0);  

/* Second KDL Rotation for the robot frame orientation */
KDL::Rotation rotation2 = Rotation(1, 0, 0, 0, 0, -1, 0, 1, 0);

/* Transform from the base robot frame to the left gripper */
tf::StampedTransform transform; 

/* Hold the synced point to translate all points */
geometry_msgs::Point synced_point; 

/* Flag to check if this is the first message to allow syncing */
bool first = true;  

void poseCallback(EECartImpedArm &arm, const geometry_msgs::PoseStamped::ConstPtr& stamped) {

  if (first) {
    geometry_msgs::Pose p = stamped->pose;     
    KDL::Vector hapVector = Vector(p.position.x, p.position.y, p.position.z);
    // Convert to robot coordinates   
    KDL::Vector roboVector = rotation * hapVector; 
    synced_point.x = transform.getOrigin().x() - roboVector.x();
    synced_point.y = transform.getOrigin().y() - roboVector.y();
    synced_point.z = transform.getOrigin().z() - roboVector.z();
    first = false; 
    }

  ee_cart_imped_msgs::EECartImpedGoal traj;  

  geometry_msgs::Pose currPose = stamped->pose; 
  geometry_msgs::Quaternion qtn = currPose.orientation;
  geometry_msgs::Point pnt = currPose.position; 

  /* Rotation given by the haptic quaternion */
  KDL::Rotation qRotation = KDL::Rotation::Quaternion(qtn.x, qtn.y, qtn.z, qtn.w);   

  /* Transforming from haptic frame to robot frame */
  KDL::Rotation finRotation = rotation2 * qRotation; 

  /* Vector from the position of the haptic point */
  KDL::Vector pVector = Vector(pnt.x, pnt.y, pnt.z);  

  /* Transforming position from haptic frame to robot frame */
  KDL::Vector finVector = rotation * pVector; 

  /* Coordinates for the current robot Quaternion transform */
  double ox;
  double oy;
  double oz;
  double ow;

  /* Store the current quaternion of the rotation */
  finRotation.GetQuaternion (ox, oy, oz, ow);  

  
/*EECartImpedArm::addTrajectoryPoint(traj, synced_point.x, synced_point.y, synced_point.z,*/
EECartImpedArm::addTrajectoryPoint(traj, finVector.x() + synced_point.x, finVector.y() + synced_point.y, finVector.z() + synced_point.z, 
				   -0.707, 0, 0, 0.707,
                                     1000, 1000, 1000, 100, 100, 100,
                                     false, false, false, false, false,
					 false, TRAJ_RATE, "/torso_lift_link");  
  arm.startTrajectory(traj, false);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hapticListener");

  ros::NodeHandle n;

  /* The arm we will be moving using the haptics information */
  EECartImpedArm arm("l_arm_cart_imped_controller");

  ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("hapticsPose", 0, boost::bind(&poseCallback,boost::ref(arm),_1));
  
  // Move the arm to the initial position 
  ee_cart_imped_msgs::EECartImpedGoal traj1; 
  
  EECartImpedArm::addTrajectoryPoint(traj1, 0.5, 0.2, -0.1, 
				   -0.707, 0, 0, 0.707,
                    1000, 1000, 1000, 100, 100, 100,
                    false, false, false, false, false,
					 false, TRAJ_RATE, "/torso_lift_link");  
					 
  arm.startTrajectory(traj1, true);

  //Store transform information for later use

  tf::TransformListener listener;

  listener.waitForTransform("/torso_lift_link", TGT_FRAME,ros::Time(0),ros::Duration(900.0));

  try { 
      listener.waitForTransform("/torso_lift_link", TGT_FRAME,ros::Time(0),ros::Duration(3.0));  
      listener.lookupTransform("/torso_lift_link", TGT_FRAME,  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    } 

  //ros::spin();
  ros::Rate rate(1000.0); 
  while (n.ok()) {
    ros::spinOnce();
    rate.sleep(); 
  } 

  return 0;
}
