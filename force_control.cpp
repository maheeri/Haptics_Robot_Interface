#include <ros/ros.h>
#include <geometry_msgs/Wrench.h> 
#include <kdl/frames.hpp>
#include <math.h> 

#define SCALE_FACTOR 2.0
#define MAX_FORCE 4.0  

using namespace KDL;

/**
* Receive wrenches from the robot and publish the transformed and scaled
wrenches for use in the haptic frame.
*/ 

/* KDL rotation from haptics frame to robot frame (column-major) */
KDL::Rotation rotation = Rotation(0, 0, -1, -1, 0, 0, 0, 1, 0);  

/* Second KDL Rotation for the robot frame orientation */
KDL::Rotation rotation2 = Rotation(1, 0, 0, 0, 0, -1, 0, 1, 0); 

/* Contains the wrench applied to the robot */ 
typedef struct {
  double forces[3];
  double torques[3];
}  RobotWrench; 

static RobotWrench roboWrench; 

/** Collects the wrench published from the stores them in the struct
    RobotWrench */ 

void wrenchCallback(const geometry_msgs::Wrench::ConstPtr& wrench) { 
  roboWrench.forces[0] = wrench->force.x;
  roboWrench.forces[1] = wrench->force.y;
  roboWrench.forces[2] = wrench->force.z;
  roboWrench.torques[0] = wrench->torque.x;
  roboWrench.torques[1] = wrench->torque.y;
  roboWrench.torques[2] = wrench->torque.z;
}


/** Reads the wrench applied by the robot and returns a wrench containing
    that information */
geometry_msgs::Wrench readWrench () {

  /* Vector of forces from the robot */
  KDL::Vector fVector = Vector(roboWrench.forces[0], roboWrench.forces[1], roboWrench.forces[2]);

 /* Vector of torques from the robot */
  KDL::Vector tVector = Vector(roboWrench.torques[0], roboWrench.torques[1], roboWrench.torques[2]);

  KDL::Wrench robotsWrench = Wrench(fVector, tVector);

  KDL::Wrench hapticWrench = rotation.Inverse() * robotsWrench;
  //(might need a different transform for the torques)  

  geometry_msgs::Wrench wrench_msg; 
 
  double x_force = hapticWrench.force.x()/SCALE_FACTOR;
  double y_force = hapticWrench.force.y()/SCALE_FACTOR;
  double z_force = hapticWrench.force.z()/SCALE_FACTOR;
  double force_mag = sqrt(pow(x_force, 2) + pow(y_force, 2) + pow(z_force, 2)); 

  if (force_mag >= MAX_FORCE) { // scale back if this too much 
    x_force = x_force * (force_mag/MAX_FORCE);
    y_force = y_force * (force_mag/MAX_FORCE);
    z_force = z_force * (force_mag/MAX_FORCE);
  }

  wrench_msg.force.x = x_force; 
  wrench_msg.force.y = y_force; 
  wrench_msg.force.z = z_force; 
  wrench_msg.torque.x = hapticWrench.torque.x();
  wrench_msg.torque.y = hapticWrench.torque.y();
  wrench_msg.torque.z = hapticWrench.torque.z();

  return wrench_msg; 
}


int main(int argc, char **argv)
{
  
  /* Subscribe to receive wrenches */
  ros::init(argc, argv, "wrenchListener");

  ros::NodeHandle n;

  ros::Subscriber wrenchSub = n.subscribe("/l_arm_cart_imped_controller/wrenchApplied", 1000, wrenchCallback);

  /* Publish wrenches to be received by the haptic device */
  ros::init(argc, argv, "wrenchPublisher");

  ros::NodeHandle n1; 

  ros::Publisher wrenchPub = n1.advertise<geometry_msgs::Wrench>("roboWrench", 1000); 

  ros::Rate rate(1000.0); 
  while (n.ok()) {
    ros::spinOnce();
    geometry_msgs::Wrench wrench_msg = readWrench();
    wrenchPub.publish(wrench_msg);
    rate.sleep(); 
  } 

  return 0;
}
