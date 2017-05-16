#include "ros/ros.h"
#include "gazebo_msgs/ApplyBodyWrench.h"
#include <cstdlib>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>


geometry_msgs::WrenchStamped f1;
geometry_msgs::Wrench f11;
geometry_msgs::WrenchStamped f2;
geometry_msgs::Wrench f22;

void chatterCallbackLR(const geometry_msgs::WrenchStamped msg1)
{ 
  f1.wrench.force.x = msg1.wrench.force.x;
  f1.wrench.force.y = msg1.wrench.force.y;
  f1.wrench.force.z = msg1.wrench.force.z;

  /*f11.force.x = msg1.wrench.force.x;
  f11.force.y = msg1.wrench.force.y;
  f11.force.z = msg1.wrench.force.z;*/
  
  
}
void chatterCallbackLT(const geometry_msgs::WrenchStamped msg2)
{
  f2.wrench.force.x = msg2.wrench.force.x;
  f2.wrench.force.y = msg2.wrench.force.y;
  f2.wrench.force.z = msg2.wrench.force.z;

  /*f22.force.x = msg2.wrench.force.x;
  f22.force.y = msg2.wrench.force.y;
  f22.force.z = msg2.wrench.force.z;*/
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "apply_body_wrench_table_client");
  

  ros::NodeHandle n;
  ros::ServiceClient clientLR = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("gazebo/apply_body_wrench");

  ros::ServiceClient clientLT = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("gazebo/apply_body_wrench");
 /*
  ros::Publisher pubLR = n.advertise<geometry_msgs::WrenchStamped>("fuerza/hander3", 1000);
  ros::Publisher pubLT = n.advertise<geometry_msgs::WrenchStamped>("fuerza/hander4", 1000);
	*/
  ros::Subscriber subLR = n.subscribe("real_sensor1", 1000, chatterCallbackLR);

  ros::Subscriber subLT = n.subscribe("real_sensor2", 1000, chatterCallbackLT);


  // Initialize the force_torque of Left robot arm
  
  gazebo_msgs::ApplyBodyWrench srv_LR;
  
  srv_LR.request.body_name ="hander4";
  srv_LR.request.reference_frame = "world";
  srv_LR.request.reference_point.x=0;
  srv_LR.request.reference_point.y = 0;
  srv_LR.request.reference_point.z =0;
  srv_LR.request.wrench.force.x = 0;
  srv_LR.request.wrench.force.y = 0;
  srv_LR.request.wrench.force.z = 0;
  srv_LR.request.wrench.torque.x= 0;
  srv_LR.request.wrench.torque.y = 0;
  srv_LR.request.wrench.torque.z = 0;
  srv_LR.request.start_time.sec = 0;
  srv_LR.request.start_time.nsec =0;
  srv_LR.request.duration.sec = 10;
  srv_LR.request.duration.nsec = 0;
  
  
  // Initialize the force_torque of Left table arm
  
  gazebo_msgs::ApplyBodyWrench srv_LT;
  
  srv_LT.request.body_name ="hander3";
  srv_LT.request.reference_frame = "world";
  srv_LT.request.reference_point.x=0;
  srv_LT.request.reference_point.y = 0;
  srv_LT.request.reference_point.z =0;
  srv_LT.request.wrench.force.x = 0;
  srv_LT.request.wrench.force.y = 0;
  srv_LT.request.wrench.force.z = 0;
  srv_LT.request.wrench.torque.x= 0;
  srv_LT.request.wrench.torque.y = 0;
  srv_LT.request.wrench.torque.z = 0;
  srv_LT.request.start_time.sec = 0;
  srv_LT.request.start_time.nsec =0;
  srv_LT.request.duration.sec = 10;
  srv_LT.request.duration.nsec = 0;

 // Create the msg to send
 /*
  geometry_msgs::Wrench force_LR;

  force_LR.force.x = 0;
  force_LR.force.y = 0;
  force_LR.force.z = 0;
  force_LR.torque.x = 0;
  force_LR.torque.y = 0;
  force_LR.torque.z = 0;

  geometry_msgs::Wrench force_LT;

  force_LT.force.x = 0;
  force_LT.force.y = 0;
  force_LT.force.z = 0;
  force_LT.torque.x = 0;
  force_LT.torque.y = 0;
  force_LT.torque.z = 0;
  */

  ros::Rate loop_rate(9000);



  int count = 0;
  while (ros::ok()){

   srv_LR.request.wrench.force.x = -f1.wrench.force.z;
    srv_LR.request.wrench.force.y = f1.wrench.force.x;
    srv_LR.request.wrench.force.z = -2*f1.wrench.force.y;

    srv_LT.request.wrench.force.x = -f2.wrench.force.z;
    srv_LT.request.wrench.force.y = f2.wrench.force.x;
    srv_LT.request.wrench.force.z = -2*f2.wrench.force.y;
/*
    force_LR.force.x = -f1.wrench.force.z;
    force_LR.force.y = f1.wrench.force.x;
    force_LR.force.z = -2*f1.wrench.force.y;

    force_LT.force.x = -f2.wrench.force.z;
    force_LT.force.y = f2.wrench.force.x;
    force_LT.force.z = -2*f2.wrench.force.y;

    pubLR.publish(force_LR);
    pubLT.publish(force_LT);
*/
    
    if (clientLR.call(srv_LR))
  {
    std::cout << "guay!!" << std::endl;
  }
  else
  {
     std::cout << "Noooooo!"<< std::endl;
    return 1;
  }

       if (clientLT.call(srv_LT))
  {
    //std::cout << "SIII2!" << std::endl;
  }
  else
  {
     std::cout << "Noooooo2!"<< std::endl;
    return 1;
  }

   ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
	
}
