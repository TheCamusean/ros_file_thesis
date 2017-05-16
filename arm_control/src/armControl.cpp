#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "sensor_msgs/JointState.h"

//#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>

// Helpers function to convert between
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

#include <math.h>



 //Julen Addition

#include <iDynTree/Sensors/PredictSensorsMeasurements.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/SixAxisFTSensor.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/Model.h>

struct EigenRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
    }

    void random()
    {
        world_H_base.setIdentity();
        jointPos.setRandom();
        baseVel.setRandom();
        jointVel.setRandom();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.8;
    }

    Eigen::Matrix4d world_H_base;
    Eigen::VectorXd jointPos;
    Eigen::Matrix<double,6,1> baseVel;
    Eigen::VectorXd jointVel;
    Eigen::Vector3d gravity;
};

struct iDynTreeRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
    }

    iDynTree::Transform world_H_base;
    iDynTree::VectorDynSize jointPos;
    iDynTree::Twist         baseVel;
    iDynTree::VectorDynSize jointVel;
    iDynTree::Vector3       gravity;
};
class jointInfo{

public:
  sensor_msgs::JointState states;
  void stateUpdate(const sensor_msgs::JointState msg);
};

void jointInfo::stateUpdate(const sensor_msgs::JointState msg)
{
  this->states = msg;

}


void positionControl(){

	double DOFsize = 29;
	double Gkp = 10;
	double Gkv = 2;
	Eigen::VectorXd q_des;
	Eigen::VectorXd qDot_des;
    Eigen::MatrixXd Kp;
	Kp = Eigen::MatrixXd::Identity(DOFsize,DOFsize)*Gkp;
	Eigen::MatrixXd Kv;
	Kv = Eigen::MatrixXd::Identity(DOFsize,DOFsize)*Gkv;




}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "armControl");


  ros::NodeHandle n;
  jointInfo jointState_var;
  ros::Subscriber sub = n.subscribe("joint_states", 1000, &jointInfo::stateUpdate, &jointState_var);

   // iDynTree stuff initialization

   iDynTree::KinDynComputations kinDynComp;
  
   iDynTree::ModelLoader mdlLoader;
   iDynTree::Model modelRobot;

    bool ok = mdlLoader.loadModelFromFile("/home/julen/catkin_ws/src/waistControl/robotModel/coman.urdf");
    //bool ok = mdlLoader.loadModelFromFile("/home/julen/iit-coman-ros-pkg/coman_urdf/urdf/coman_robot.urdf");

    if( !ok )
    {
        std::cout << "KinDynComputationsWithEigen: impossible to load model "  << std::endl;
        //return EXIT_FAILURE;
    }

    // Create a KinDynComputations class from the model
    ok = kinDynComp.loadRobotModel(mdlLoader.model());


    if( !ok )
    {
        std::cout << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        //return EXIT_FAILURE;
    }

  const iDynTree::Model & model = kinDynComp.model();
  iDynTree::FreeFloatingGeneralizedTorques gGF;
  gGF.resize(model);



// Compute dynamics??
  EigenRobotState eigRobotState;
  eigRobotState.resize(model.getNrOfDOFs());
  eigRobotState.random();

  iDynTreeRobotState idynRobotState;

  idynRobotState.resize(model.getNrOfDOFs());
  iDynTree::fromEigen(idynRobotState.world_H_base,eigRobotState.world_H_base);
  iDynTree::toEigen(idynRobotState.jointPos) = eigRobotState.jointPos;
  iDynTree::fromEigen(idynRobotState.baseVel,eigRobotState.baseVel);
  toEigen(idynRobotState.jointVel) = eigRobotState.jointVel;
  toEigen(idynRobotState.gravity)  = eigRobotState.gravity;

  kinDynComp.setRobotState(idynRobotState.world_H_base,idynRobotState.jointPos,
                             idynRobotState.baseVel,idynRobotState.jointVel,idynRobotState.gravity);





  kinDynComp.generalizedGravityForces(gGF);
  kinDynComp.generalizedBiasForces(gGF);
  //std::cout<<gGF.jointTorques().toString() <<std::endl;
   
  double DOFsize = model.getNrOfDOFs();


  ros::Rate loop_rate(100);
  while (ros::ok())
  {
  	modelRobot = mdlLoader.model();
    std::vector<double> v = {13,14,22,23,10,9,11,12,20,19,21,24,25,7,8,18,26,4,3,5,6,16,15,17,27,28,0,1,2};
    if(jointState_var.states.name.size() == DOFsize){
          for(unsigned int i =0; i < DOFsize; i++ ) {
              eigRobotState.jointPos(v[i])=jointState_var.states.position[i];
              eigRobotState.jointVel(v[i])=jointState_var.states.velocity[i]; 
          }
    iDynTree::toEigen(idynRobotState.jointPos) = eigRobotState.jointPos;
    toEigen(idynRobotState.jointVel) = eigRobotState.jointVel;

    kinDynComp.setRobotState(idynRobotState.world_H_base,idynRobotState.jointPos,
                                   idynRobotState.baseVel,idynRobotState.jointVel,idynRobotState.gravity);
    kinDynComp.generalizedGravityForces(gGF);

    positionControl();




	};


  	


    
    ros::spinOnce();

    loop_rate.sleep();
 
  }

  return 0;
}
