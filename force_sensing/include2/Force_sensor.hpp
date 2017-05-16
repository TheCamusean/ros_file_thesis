/* Author Julen Urain
 *
 *
 *
 */

 #pragma once

// RTT header files. Might missing some or some be unused
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>

//For port_names
#include <vector>
#include <string>


// Eigen headers
#include <Eigen/Core>
// C headers
#include <cstdlib>

//optoforce libraries
#include "MinimalOpto.hpp"
#include "Force_vector.hpp"

#include <fstream> 


 //#include <rst-rt/dynamics/Forces.hpp>

 #include <rtt/RTT.hpp>


#include <std_msgs/Float32.h>

class Force_sensor: public RTT::TaskContext {
public:
    Force_sensor(std::string const & name);
     // ComplianceController(iDynTree::KinDynComputations & kinDynComp);
    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();


    void Print_force();
    void preparePorts();
    void print_ports();
    void weight_up(double W);

    


private:

    bool Bool_param;
    double Weight;
    vector<string> port_names;
    MinimalOpto opto;
    MinimalOpto opto2;

    
    RTT::OutputPort<rstrt::dynamics::Forces> out_force_port;
    rstrt::dynamics::Forces F;

    RTT::OutputPort<rstrt::dynamics::Forces> out_force_port2;
    rstrt::dynamics::Forces F2;

    RTT::OutputPort<std_msgs::Float32> out_force11;
    RTT::OutputPort<std_msgs::Float32> out_force12;
    RTT::OutputPort<std_msgs::Float32> out_force13;
RTT::OutputPort<std_msgs::Float32> out_force21;
RTT::OutputPort<std_msgs::Float32> out_force22;
RTT::OutputPort<std_msgs::Float32> out_force23;


    


};
