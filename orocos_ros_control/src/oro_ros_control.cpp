#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"




/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
sensor_msgs::JointState torque;
sensor_msgs::JointState torque_R;

void chatterCallback(const sensor_msgs::JointState torque_act)
{
  std::cout << torque << std::endl;

  torque =torque_act;

}

void chatterCallback_R(const sensor_msgs::JointState torque_act_R)
{
  std::cout << torque_R << std::endl;

  torque_R =torque_act_R;

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "oro_ros_control");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joint_torque_L", 1000, chatterCallback);

  ros::Subscriber sub_R = n.subscribe("joint_torque_R", 1000, chatterCallback_R);

  // Force publisher for Left Arm (LShSag , LShLat, LShYaw, LElbj, ...) 

  ros::Publisher LElbj_pub  = n.advertise<std_msgs::Float64>("LElbj_controller/command", 1000);
  ros::Publisher LShSag_pub = n.advertise<std_msgs::Float64>("LShSag_controller/command", 1000);
  ros::Publisher LShLat_pub = n.advertise<std_msgs::Float64>("LShLat_controller/command", 1000);
  ros::Publisher LShYaw_pub = n.advertise<std_msgs::Float64>("LShYaw_controller/command", 1000);
  ros::Publisher LForearmPlate_pub = n.advertise<std_msgs::Float64>("LForearmPlate_controller/command", 1000);
  ros::Publisher LWrj1_pub = n.advertise<std_msgs::Float64>("LWrj1_controller/command", 1000);
  ros::Publisher LWrj2_pub = n.advertise<std_msgs::Float64>("LWrj2_controller/command", 1000);

  // Force publisher for Right Arm (RShSag , RShLat, RShYaw, RElbj, ...) 

  ros::Publisher RElbj_pub  = n.advertise<std_msgs::Float64>("RElbj_controller/command", 1000);
  ros::Publisher RShSag_pub = n.advertise<std_msgs::Float64>("RShSag_controller/command", 1000);
  ros::Publisher RShLat_pub = n.advertise<std_msgs::Float64>("RShLat_controller/command", 1000);
  ros::Publisher RShYaw_pub = n.advertise<std_msgs::Float64>("RShYaw_controller/command", 1000);
  ros::Publisher RForearmPlate_pub = n.advertise<std_msgs::Float64>("RForearmPlate_controller/command", 1000);
  ros::Publisher RWrj1_pub = n.advertise<std_msgs::Float64>("RWrj1_controller/command", 1000);
  ros::Publisher RWrj2_pub = n.advertise<std_msgs::Float64>("RWrj2_controller/command", 1000);

  

 // initialization of torque info Left;
   for(unsigned int i = 0; i < 7; ++i)
    {
        torque.position.push_back(0.0);
        torque.effort.push_back(0.0);
        torque.velocity.push_back(0.0);
    }

 // initialization of torque info Right;
   for(unsigned int i = 0; i < 7; ++i)
    {
        torque_R.position.push_back(0.0);
        torque_R.effort.push_back(0.0);
        torque_R.velocity.push_back(0.0);
    }


	ros::Rate loop_rate(1000);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
	//Torque variables initialization
  	std_msgs::Float64 LElbj_var;
	std_msgs::Float64 LShSag_var;
	std_msgs::Float64 LShLat_var;
	std_msgs::Float64 LShYaw_var;
	std_msgs::Float64 LForearmPlate_var;
	std_msgs::Float64 LWrj1_var;
	std_msgs::Float64 LWrj2_var;

	//Torque variables initialization
  	std_msgs::Float64 RElbj_var;
	std_msgs::Float64 RShSag_var;
	std_msgs::Float64 RShLat_var;
	std_msgs::Float64 RShYaw_var;
	std_msgs::Float64 RForearmPlate_var;
	std_msgs::Float64 RWrj1_var;
	std_msgs::Float64 RWrj2_var;


 	//Give the values

	LShSag_var.data = torque.effort[0];
	LShLat_var.data = torque.effort[1];
	LShYaw_var.data = torque.effort[2];
	LElbj_var.data =  torque.effort[3];

	// divided by 10 in order to have the same proportional gain of the YARP controller
	LForearmPlate_var.data = 0.1* torque.effort[4];
	LWrj1_var.data =  0.1* torque.effort[5];
	LWrj2_var.data =  0.1* torque.effort[6];

	//Give the values

	RShSag_var.data = torque_R.effort[0];
	RShLat_var.data = torque_R.effort[1];
	RShYaw_var.data = torque_R.effort[2];
	RElbj_var.data =  torque_R.effort[3];

	// divided by 10 in order to have the same proportional gain of the YARP controller
	RForearmPlate_var.data = 0.1* torque_R.effort[4];
	RWrj1_var.data =  0.1* torque_R.effort[5];
	RWrj2_var.data =  0.1* torque_R.effort[6];


	// Send torque to Gazebo enviroment
	
	LShSag_pub.publish(LShSag_var);
	LShLat_pub.publish(LShLat_var);
	LShYaw_pub.publish(LShYaw_var);
	LElbj_pub.publish(LElbj_var);
	LForearmPlate_pub.publish(LForearmPlate_var);
	LWrj1_pub.publish(LWrj1_var);
	LWrj2_pub.publish(LWrj2_var);


	// Send torque to Gazebo enviroment
	
	RShSag_pub.publish(RShSag_var);
	RShLat_pub.publish(RShLat_var);
	RShYaw_pub.publish(RShYaw_var);
	RElbj_pub.publish(RElbj_var);
	RForearmPlate_pub.publish(RForearmPlate_var);
	RWrj1_pub.publish(RWrj1_var);
	RWrj2_pub.publish(RWrj2_var);

    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
