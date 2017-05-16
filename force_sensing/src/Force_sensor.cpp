/* Author: Julen Urain
 * Date:   22/02/2017
 *
 *
 *
 */
#include "Force_sensor.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>

//For port_names
#include <vector>
#include <string>

#include "MinimalOpto.hpp"


#include <rtt/Operation.hpp>
#include </home/julen/ws/underlay/src/rtt_ros_integration/rtt_roscomm/include/rtt_roscomm/rtt_rostopic.h>
#include </home/julen/ws/underlay/src/rtt_ros_integration/rtt_rosclock/include/rtt_rosclock/rtt_rosclock.h>
#include <rtt/OperationCaller.hpp>



/*Written by Mostafa ...*/
vector<string> find_port_by_serial(string serial, string typestring, bool get_all = false, uint max_num = 10)
{
  vector<string> port_name;
  for (uint i = 0; i < max_num; i++)
  {
    stringstream ss;
    ss << "udevadm info -a -n /dev/tty" << typestring << i << " 2>/dev/null | grep 'idVendor' | head -n1 > /tmp/find_port_by_serial_result.txt";
    system(ss.str().c_str());
    fstream resfile("/tmp/find_port_by_serial_result.txt", fstream::in);
    string content;
    resfile >> content;
    size_t found = content.find(serial);
    if(found != string::npos)
    {
      stringstream portnamess;
      portnamess << "/dev/tty" << typestring << i;
      port_name.push_back(portnamess.str());
      if(!get_all)
      {
	break;
      }
    }
  }

  return port_name;
}
/*until here*/


std::ofstream outputFile1;
std::ofstream outputFile2;
double old_vel = 0;


Force_sensor::Force_sensor(std::string const & name) : port_names(find_port_by_serial("04d8", "ACM", true)), RTT::TaskContext(name),opto(port_names[0], MinimalOpto::DataFreq::HZ100), opto2(port_names[1], MinimalOpto::DataFreq::HZ100) {
    // constructor:
    addOperation("Print_force", &Force_sensor::Print_force, this, RTT::ClientThread).doc("Me da la fuerza");
    addOperation("preparePorts", &Force_sensor::preparePorts, this, RTT::ClientThread).doc("Get the ports ready");
    addOperation("prin_ports", &Force_sensor::print_ports, this, RTT::ClientThread).doc("print the ports");
    addOperation("weight_up", &Force_sensor::weight_up, this, RTT::ClientThread).doc("Increase the z axis force");

    
}

bool Force_sensor::configureHook() {

    //this -> preparePorts();
    bool inizialized = false;

}

bool Force_sensor::startHook(){


  cout << "inichiamos" << endl;
  Bool_param = 0;
  
  outputFile1.open ("Fuerza_sensor1_fastwalkJe.txt");
  outputFile2.open ("Fuerza_sensor2_fastwalkJe.txt");


  out_force11.createStream(rtt_roscomm::topic("forces11"));
  out_force12.createStream(rtt_roscomm::topic("forces12"));
 
  out_force21.createStream(rtt_roscomm::topic("FSUM00forces21"));
  out_force22.createStream(rtt_roscomm::topic("FDIFF00forces22"));


   // Anterior Direction
  out_force13.createStream(rtt_roscomm::topic("forces13"));
  out_force23.createStream(rtt_roscomm::topic("forces23"));
  
  
  
  return true;

}

void Force_sensor::updateHook(){

    double f[3] = {0, 0, 0};
    opto.GetComputedForces(f);

    if(Bool_param ==1){f[2]=Weight;}
    
    
    F.forces[0] = f[0];
    F.forces[1] = f[1];
    F.forces[2] = f[2];
    out_force_port.write(F);
    std_msgs::Float32 x;

    x.data = f[0];
    x.data = old_vel;
    out_force11.write(x);
    
    
    out_force12.write(x);
    x.data = f[2];
    out_force13.write(x);

    double f2[3] = {0, 0, 0};
    opto2.GetComputedForces(f2);

    if(Bool_param ==1){f2[2]=Weight;}
    
    
    F2.forces[0] = f2[0];
    F2.forces[1] = f2[1];
    F2.forces[2] = f2[2];
    out_force_port2.write(F2);

    x.data = f2[0];
    out_force21.write(x);
    x.data = f[1];
    out_force22.write(x);
    x.data = f2[2];
    out_force23.write(x);

    old_vel = old_vel +(f[2]+f2[2])*0.001/3;
    

   this -> Print_force();


}

void Force_sensor::stopHook() {
    // stops the component (update hook wont be  called anymore)
}
void Force_sensor::cleanupHook() {
    // stops the component (update hook wont be  called anymore)
}




void Force_sensor::print_ports(){

      cout<< port_names[0] << "  "<< port_names[1]<<endl;


}


void Force_sensor::Print_force(){

   
    
    //cout << "aqui llego" << endl;
    //cout << F.forces[0] << "\t" << F.forces[1] << "\t" << F.forces[2] << endl;
    outputFile1 <<  F.forces[0] << "\t" << F.forces[1] << "\t" << F.forces[2];
    outputFile1 << "\n";
    //cout << F2.forces[0] << "\t" << F2.forces[1] << "\t" << F2.forces[2] << endl;
    outputFile2 << F2.forces[0] << "\t" << F2.forces[1] << "\t" << F2.forces[2] <<"\n";
    //cout << "aqui bajo" << endl;
    



}
void Force_sensor::preparePorts(){


  F=rstrt::dynamics::Forces();
  F.forces.setZero();
  out_force_port.setName("out_force_port");
  out_force_port.doc("Output port for sending force");
  out_force_port.setDataSample(F);
  ports()->addPort(out_force_port);

  F2=rstrt::dynamics::Forces();
  F2.forces.setZero();
  out_force_port2.setName("out_force_port2");
  out_force_port2.doc("Output port 2 for sending force");
  out_force_port2.setDataSample(F2);
  ports()->addPort(out_force_port2);


  out_force11.setName("out_force11");
  out_force11.doc("lalalele");
  std_msgs::Float32 x;
  x.data = 0;
  out_force11.setDataSample(x);
  ports()->addPort(out_force11);

  out_force12.setName("out_force12");
  out_force12.doc("lalalele");

  out_force12.setDataSample(x);
  ports()->addPort(out_force12);

  out_force13.setName("out_force13");
  out_force13.doc("lalalele");

  out_force13.setDataSample(x);
  ports()->addPort(out_force13);

  out_force21.setName("out_force21");
  out_force21.doc("lalalele");

  out_force21.setDataSample(x);
  ports()->addPort(out_force21);

  out_force22.setName("out_force22");
  out_force22.doc("lalalele");
  out_force22.setDataSample(x);
  ports()->addPort(out_force22);

  out_force23.setName("out_force23");
  out_force23.doc("lalalele");

  out_force23.setDataSample(x);
  ports()->addPort(out_force23);



}

void Force_sensor::weight_up(double W){

	Weight = W;
	Bool_param = 1;




}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(Force_sensor)
