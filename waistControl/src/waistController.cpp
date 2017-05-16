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


iDynTree::FreeFloatingGeneralizedTorques gravityCompensation()
{


}

iDynTree::KinDynComputations kinDynComp;

iDynTree::Model modelRobot;
double fzr = 10;
double fzl = 10;
Eigen::VectorXd des_tor(35);
Eigen::Vector3d CoM_vec_int(0,0,0);

void stabilityControl()
{

// I am gonna try a super simple approach in order to calculate the position of the CoM depending on the WaistSag

// Compute the CoM --> Without modifying 

  iDynTree::Position CoM;

  CoM = kinDynComp.getCenterOfMassPosition();

  //std::cout << CoM.toString()<< std::endl;

  // Jacobian of the CoM;
  iDynTree::MatrixDynSize JCoM;
  kinDynComp.getCenterOfMassJacobian(JCoM);

 
  // Mass Matrix
  iDynTree::MatrixDynSize M;
  kinDynComp.getFreeFloatingMassMatrix(M);
 // std::cout << kinDynComp.getFloatingBase() << std::endl;

  


  // Total Mass ----> New CoM considering the Force info
  iDynTree::VectorDynSize inertialParam;
  modelRobot.getInertialParameters(inertialParam);
  double totMass = 0;
  for(unsigned int i =0; i < modelRobot.getNrOfLinks(); i++ ){ // 36 links ---> 360 parameters
        totMass = totMass + inertialParam.data()[10*i];

  }
  //std::cout << modelRobot.getNrOfLinks() << std::endl;
  //std::cout << inertialParam.size() << std::endl;
  //std::cout << totMass << std::endl;


  // CoM considering the force effect on wrist;


  // ##################### Left Wrist #########################
  iDynTree::Transform TLWrWa;

  TLWrWa = kinDynComp.getRelativeTransform("Waist","LWrMot3");


  Eigen::MatrixXf  TLWrWa_eig(4,4);
  
  for(unsigned int i=0; i< 4;i++){
    for(unsigned int j = 0; j<4;j++){

      TLWrWa_eig(i,j)  = TLWrWa.asHomogeneousTransform().getVal(i,j);

    }
  }
  

  Eigen::VectorXf  forceLWa(4);
  Eigen::VectorXf  forceLWr(4);
 
  forceLWr << 0,0,0,1;
  //std::cout << TWrWa.asHomogeneousTransform().toString() << std::endl;
  //std::cout << TWrWa_eig<<std::endl;
  forceLWa = TLWrWa_eig*forceLWr;



  //#################### Right Wrist #####################
  iDynTree::Transform TRWrWa;

  TRWrWa = kinDynComp.getRelativeTransform("Waist","RWrMot3");


  Eigen::MatrixXf  TRWrWa_eig(4,4);
  
  for(unsigned int i=0; i< 4;i++){
    for(unsigned int j = 0; j<4;j++){

      TRWrWa_eig(i,j)  = TRWrWa.asHomogeneousTransform().getVal(i,j);

    }
  }
  

  Eigen::VectorXf  forceRWa(4);
  Eigen::VectorXf  forceRWr(4);
 
  forceRWr << 0,0,0,1;
  //std::cout << TWrWa.asHomogeneousTransform().toString() << std::endl;
  //std::cout << TWrWa_eig<<std::endl;
  forceRWa = TRWrWa_eig*forceRWr;




  //std::cout << forceWa.data()[0] << " "<< forceWa.data()[1] <<" "<< forceWa.data()[2] << " "<< forceWa.data()[3]  << std::endl;
  
  double CoM_wFx;
  double CoM_wFy;
  double CoM_wFz;

  CoM_wFx = CoM.getVal(0) * totMass/(totMass+fzl/9.8 +fzr/9.8) + (fzl/9.8*forceLWa.data()[0] + fzr/9.8*forceRWa.data()[0])/(totMass+fzl/9.8+fzr/9.8);
  CoM_wFy = CoM.getVal(1) * totMass/(totMass+fzl/9.8 +fzr/9.8) + (fzl/9.8*forceLWa.data()[1] + fzr/9.8*forceRWa.data()[1])/(totMass+fzl/9.8+fzr/9.8);
  CoM_wFz = CoM.getVal(2) * totMass/(totMass+fzl/9.8 +fzr/9.8) + (fzl/9.8*forceLWa.data()[2] + fzr/9.8*forceRWa.data()[2])/(totMass+fzl/9.8+fzr/9.8);


 std::cout << CoM_wFx<<" " << CoM_wFy << " " << CoM_wFz << " " << std::endl;
 std::cout << CoM.toString()<< std::endl;


 // Inverse Jacobian

//double data[3][34];
//Eigen::Map<Eigen::Matrix<double,3,34,Eigen::RowMajor> > JCoM_eig(data[0]);
Eigen::MatrixXd JCoM_eig(JCoM.rows(),JCoM.cols());

for(unsigned int i=0; i< JCoM.rows();i++){
  for(unsigned int j = 0; j<JCoM.cols();j++){

    JCoM_eig(i,j)  = JCoM.getVal(i,j);

  }
}
//std::cout << JCoM_eig<< std::endl;
Eigen::MatrixXd invJCoM;
invJCoM = JCoM_eig.transpose();
//std::cout << invJCoM<< std::endl;

//Desired Torque

Eigen::Vector3d CoM_vec(CoM_wFx,CoM_wFy,0);


Eigen::MatrixXd M_eig(M.rows(),M.cols());

//std::cout << JCoM.rows()<< std::endl;
//std::cout << JCoM.cols()<< std::endl;
//toEigen(JCoM) = JCoM_eig;
for(unsigned int i=0; i< M.rows();i++){
  for(unsigned int j = 0; j<M.cols();j++){

    M_eig(i,j)  = M.getVal(i,j);

  }
}



//std::cout << invJCoM.rows() << "  " << invJCoM.cols() << std::endl;
//std::cout << CoM_vec.rows() << "  " << CoM_vec.cols() << std::endl;
//std::cout << des_tor.rows() << "  " << des_tor.cols() << std::endl;
//std::cout << M_eig.rows() << "  " << M_eig.cols() << std::endl;
Eigen::MatrixXd Kp;
Eigen::MatrixXd Ki;
Kp = Eigen::MatrixXd::Identity(3,3)*500;
Ki = Eigen::MatrixXd::Identity(3,3)*0.1;
CoM_vec_int = CoM_vec_int + CoM_vec/100;



des_tor = -M_eig*invJCoM*(Kp*CoM_vec + Ki*CoM_vec_int);

std::cout << CoM_vec_int << "  hola " <<CoM_vec << std::endl;





  

  
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "waistController");

  ros::NodeHandle n;
  jointInfo jointState_var;
  ros::Subscriber sub = n.subscribe("joint_states", 1000, &jointInfo::stateUpdate, &jointState_var);

  ros::Publisher waistYawTorquePub = n.advertise<std_msgs::Float64>("WaistYaw_controller/command", 1000);
  ros::Publisher waistSagTorquePub = n.advertise<std_msgs::Float64>("WaistSag_controller/command", 1000);

  ros::Rate loop_rate(100);

  // iDynTree stuff initialization

  //iDynTree::KinDynComputations kinDynComp;
  
  iDynTree::ModelLoader mdlLoader;

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
  int count = 0;
  while (ros::ok())
  {

    //std::cout << mdlLoader.model().toString() << std::endl;

    //iDynTree::VectorDynSize inertialParam;

    modelRobot = mdlLoader.model();
    
    


    //std::cout << jointState_var.states << std::endl;
    //std::cout << jointState_var.states.name.size() << "  " << DOFsize << std::endl;
    //std::cout << kinDynComp.getDescriptionOfDegreesOfFreedom() << std::endl;
    std::vector<double> v = {13,14,22,23,10,9,11,12,20,19,21,24,25,7,8,18,26,4,3,5,6,16,15,17,27,28,0,1,2};
    if(jointState_var.states.name.size() == DOFsize){
          for(unsigned int i =0; i < DOFsize; i++ ) {
              eigRobotState.jointPos(v[i])=jointState_var.states.position[i];
              eigRobotState.jointVel(v[i])=jointState_var.states.velocity[i]; 
             // eigRobotState.baseVel (i)=0;
          }
          iDynTree::toEigen(idynRobotState.jointPos) = eigRobotState.jointPos;
          toEigen(idynRobotState.jointVel) = eigRobotState.jointVel;

           kinDynComp.setRobotState(idynRobotState.world_H_base,idynRobotState.jointPos,
                                   idynRobotState.baseVel,idynRobotState.jointVel,idynRobotState.gravity);
           kinDynComp.generalizedGravityForces(gGF);
           //std::cout<<gGF.jointTorques().toString() <<std::endl;

           stabilityControl();

           std_msgs::Float64 x;
           x.data = gGF.jointTorques().getVal(2) + 5 * (0 -jointState_var.states.position[28]);
           waistYawTorquePub.publish(x);

           std_msgs::Float64 y;
           y.data = gGF.jointTorques().getVal(1)+   des_tor[1] + 5 * ( -jointState_var.states.position[27]);
           std::cout << "Bias Forces Torque" << gGF.jointTorques().getVal(1)<< "stabiliy" << des_tor[1] << "position control" << 5 * ( -jointState_var.states.position[27]) << std::endl;
           waistSagTorquePub.publish(y);
    };


    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
