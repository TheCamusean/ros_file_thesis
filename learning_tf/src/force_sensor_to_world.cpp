#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/WrenchStamped.h>

geometry_msgs::WrenchStamped Force_vec_LF;

void force_sensing(const geometry_msgs::WrenchStamped msg)
{ 
    std::cout << "msg" << std::endl;
    Force_vec_LF.wrench.force.x = msg.wrench.force.x;
    Force_vec_LF.wrench.force.y = msg.wrench.force.y;	
    Force_vec_LF.wrench.force.z = msg.wrench.force.z;
    
}


int main(int argc, char** argv){
  ros::init(argc, argv, "force_sensor_to_world");

  ros::NodeHandle node;

  ros::Subscriber force_sens_sus = node.subscribe("coman/ft_sensor/LFP", 1000,    force_sensing);

  ros::Publisher force_sens_pub = 
    node.advertise<geometry_msgs::WrenchStamped>("coman/ft_sensor/LFP_W", 10);

  
 
  ros::spin(); 
  /*tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try {
    
    listener.waitForTransform("/LForearm", "/world", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/LForearm", "/world", ros::Time(0), transform);
   
} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}
   
   tf::Vector3 V(Force_vec_LF.wrench.force.x,Force_vec_LF.wrench.force.y,Force_vec_LF.wrench.force.z);
 std::cout << Force_vec_LF.wrench.force.x << " " <<  Force_vec_LF.wrench.force.y << " " << Force_vec_LF.wrench.force.z << std::endl;
 std::cout << V[0]<<" " << V[1]<< " "<<V[2] <<std::endl;
   
  
   tf::Vector3 V_W= transform*V;
   geometry_msgs::WrenchStamped Force_vec_LF_W;
   
   Force_vec_LF_W.wrench.force.x = V_W[0];
   Force_vec_LF_W.wrench.force.y = V_W[1];
   Force_vec_LF_W.wrench.force.z = V_W[2];
   
   force_sens_pub.publish(Force_vec_LF_W);
   

    rate.sleep();
  }*/
  return 0;
  
};
