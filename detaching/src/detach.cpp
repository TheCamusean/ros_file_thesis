#include "ros/ros.h"
#include <gazebo_ros_link_attacher/Attach.h>
#include <cstdlib>
#include <geometry_msgs/WrenchStamped.h>

int breaker = 0;
void chatterCallback1(const geometry_msgs::WrenchStamped f1)
{ 
 std::cout << f1 << "quee" <<std::endl;
 if(abs(f1.wrench.force.x) + abs(f1.wrench.force.y) +abs(f1.wrench.force.z) >80){
	std::cout << "Too Much Force!!" <<std::endl;
	breaker=1;
 }
}
void chatterCallback2(const geometry_msgs::WrenchStamped f2)
{
  std::cout << f2 <<std::endl;
  if(abs(f2.wrench.force.x) + abs(f2.wrench.force.y) +abs(f2.wrench.force.z) >80){
	std::cout << "Too Much Force!!" <<std::endl;
	breaker=1;
 }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Force_controller");
  

  ros::NodeHandle n;
  ros::ServiceClient detacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");

  ros::Subscriber ft_h1 = n.subscribe("ft_sensor/hander1", 10, chatterCallback1);

  ros::Subscriber ft_h2 = n.subscribe("ft_sensor/hander2", 10, chatterCallback2);


  // Initialize the force_torque of Left robot arm
  
  gazebo_ros_link_attacher::Attach srv;
  
  srv.request.model_name_1 = "COMAN";
  srv.request.link_name_1 = "LWrMot3";
  srv.request.model_name_2 = "rrbot";
  srv.request.link_name_2 = "hander2";

  gazebo_ros_link_attacher::Attach srv2;
  
  srv2.request.model_name_1 = "COMAN";
  srv2.request.link_name_1 = "RWrMot3";
  srv2.request.model_name_2 = "rrbot";
  srv2.request.link_name_2 = "hander1";
  


  ros::Rate loop_rate(10);

  
  while (ros::ok()){
   
   if(breaker == 1){
	detacher.call(srv);
	detacher.call(srv2);
	breaker = 0;}

   	ros::spinOnce();
	loop_rate.sleep();

  }

  return 0;
	
}
