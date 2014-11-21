#include <ros/ros.h>
#include <geometry_msgs/Wrench.h> 
#include <kdl/frames.hpp>
#include <math.h> 

#define SCALE_FACTOR 8.0
#define MAX_FORCE 4.0 
#define AVG_LEN 5  

using namespace KDL;

/** Receive wrenches from the robot and publish the transformed and scaled
wrenches for use in the haptic frame. */

/* Buffer for the moving average function */
geometry_msgs::Wrench buffer[AVG_LEN]; 

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


/** Produces a moving average of the last AVG_LEN points in the 
    stream of points for smoother transitions and stores them */
void computeMovAvg() {
  double xForceSum  = 0.0;
  double yForceSum  = 0.0;
  double zForceSum  = 0.0;
  double xTorqueSum  = 0.0;
  double yTorqueSum  = 0.0;
  double zTorqueSum  = 0.0;
  for (int i = 0; i < AVG_LEN; i++) {
    xForceSum += buffer[i].force.x;
    yForceSum += buffer[i].force.y;
    zForceSum += buffer[i].force.z;
    xTorqueSum += buffer[i].torque.x;
    yTorqueSum += buffer[i].torque.y;
    zTorqueSum += buffer[i].torque.z;
  }
  roboWrench.forces[0] = xForceSum/AVG_LEN;
  roboWrench.forces[1] = yForceSum/AVG_LEN;
  roboWrench.forces[2] = zForceSum/AVG_LEN;
  roboWrench.torques[0] = xTorqueSum/AVG_LEN;
  roboWrench.torques[1] = yTorqueSum/AVG_LEN;
  roboWrench.torques[2] = zTorqueSum/AVG_LEN;
} 

/** Collects the wrench published and stores into the first location
of the buffer while moving everything else to the right */ 

void wrenchCallback(const geometry_msgs::Wrench::ConstPtr& wrench) {
  geometry_msgs::Wrench buffer_copy[AVG_LEN]; 
  std::copy(buffer, buffer + AVG_LEN, buffer_copy);  
  for (int i = 1; i < AVG_LEN; i++) { 
    buffer[i] = buffer_copy[i - 1]; 
  }
  buffer[0] = *wrench;
}


/** Reads the wrench applied by the robot and returns a wrench containing
    that information */
void publishWrench (ros::Publisher wrenchPub) {

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

  if (force_mag >= MAX_FORCE) { // scale back if this is too much 
    x_force = x_force * (force_mag/MAX_FORCE);
    y_force = y_force * (force_mag/MAX_FORCE);
    z_force = z_force * (force_mag/MAX_FORCE);
  }

  wrench_msg.force.x = -x_force; 
  wrench_msg.force.y = -y_force; 
  wrench_msg.force.z = -z_force; 
  wrench_msg.torque.x = hapticWrench.torque.x();
  wrench_msg.torque.y = hapticWrench.torque.y();
  wrench_msg.torque.z = hapticWrench.torque.z();

  wrenchPub.publish(wrench_msg); 
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

  ros::Rate rate(20.0); 
  while (n.ok()) {
    ros::spinOnce();
    computeMovAvg(); 
    publishWrench(wrenchPub);
    rate.sleep(); 
  } 

  return 0;
}
