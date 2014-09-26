#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>

#define TGT_FRAME "/l_gripper_tool_frame"

using namespace KDL; 

/**
* Receive pose and orientation information from ROS topics and send it on to ee_cart for
* simulator control.
*/

/* Contains the 10 poses received so far */ 
geometry_msgs::Pose poseBuffer[10]; 

/* Pointer into the pose buffer to see if it is full */
int pointer; 

/* KDL rotation from haptics frame to robot frame (column-major) */
KDL::Rotation rotation = Rotation(0, 0, -1, -1, 0, 0, 0, 1, 0); 

/* Coordinates for the current robot Quaternion transform */
double ox;
double oy;
double oz;
double ow; 

/* Transform from the base robot frame to the left gripper */
tf::StampedTransform transform; 

/* Hold the synced point to translate all points */
geometry_msgs::Point synced_point; 

/* Flag to check if this is the first message to allow syncing */
bool first = true; 

void poseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
  poseBuffer[pointer] = *pose;    

  /*if (first) {
    std::cout << "fin_x " << (finVector.x() + synced_point.x) << std::endl;
    std::cout << "fin_y " << (finVector.y() + synced_point.y) << std::endl;
    std::cout << "fin_z " << (finVector.z() + synced_point.z) << std::endl;
    geometry_msgs::Pose p = *pose;     
    KDL::Vector hapVector = Vector(p.position.x, p.position.y, p.position.z);
    // Convert to robot coordinates   
    KDL::Vector roboVector = rotation * hapVector; 
    synced_point.x = transform.getOrigin().x() - roboVector.x();
    synced_point.y = transform.getOrigin().y() - roboVector.y();
    synced_point.z = transform.getOrigin().z() - roboVector.z();
    first = false; 
    }*/

  if (pointer == 9) {
    EECartImpedArm arm("l_arm_cart_imped_controller");
    ee_cart_imped_msgs::EECartImpedGoal traj;
    ros::Rate rate(100.0);  
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

      std::cout << transform.getOrigin().x() << " " << finVector.y() << std::endl;
	std::cout << transform.getOrigin().y() << " " << finVector.z() << std::endl;
	std::cout << transform.getOrigin().z() << " " << finVector.x() << std::endl;

      EECartImpedArm::addTrajectoryPoint(traj, finVector.x() + transform.getOrigin().x(), finVector.y() + transform.getOrigin().y(), finVector.z() + transform.getOrigin().z(),
				     0, 0, 0, 1,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
					 false, 0.01 * (i + 1), "/torso_lift_link");
      //rate.sleep(); 
    }
    arm.startTrajectory(traj, true);
    pointer = 0; 
  } // end of if statement 
  else {
    pointer++;
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hapticListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("hapticsPose", 1000, poseCallback);

  //Store transform information for later use

  tf::TransformListener listener;

  listener.waitForTransform("/torso_lift_link", TGT_FRAME,ros::Time(0),ros::Duration(900.0));

  try {
      std::cout << "1" << std::endl;  
      listener.waitForTransform("/torso_lift_link", TGT_FRAME,ros::Time(0),ros::Duration(3.0));  
      std::cout << "2" << std::endl; 
      listener.lookupTransform("/torso_lift_link", TGT_FRAME,  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
  std::cout << "3" << std::endl; 

  ros::spin();

  return 0;
}
