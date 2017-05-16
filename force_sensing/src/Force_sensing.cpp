#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Wrench.h"

#include <sstream>

#include <iostream>
#include <fstream>

#include <vector>
#include <string>

#include "MinimalOpto.hpp"

#include <sound_play/sound_play.h>
#include <unistd.h>

#include "Keyboard.hpp"

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

void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
      sleep(t);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Force_read");


  ros::NodeHandle n;

 

  ros::Publisher F1 = n.advertise<geometry_msgs::Wrench>("fLead", 1000);
  ros::Publisher F2 = n.advertise<geometry_msgs::Wrench>("fFollow", 1000);


  ros::Rate loop_rate(1000);
  
  std::ofstream outputFile1;
  // PARAMETERS TO INTRODUCE
  outputFile1.open ("Perturbation.txt");
  double T = 0.1;

  bool Bool_param;
  double Weight;
 
  bool debug = false;
  double threshold_force = 10;
  string opto_serial = "04d8"; //"0000:00:1d.0";
  const uint optocount = 2;
  vector<vector<double> > f(optocount, vector<double>(3, 0));
  vector<MinimalOpto*> optos;
  vector<string> portnames = find_port_by_serial(opto_serial, "ACM", true);
  if(optocount != portnames.size())
  {
    cout << "Error: the expected number of optoforce sensros is " << optocount << ", but there are " <<  portnames.size() << " on /dev/ttyACM*" << endl;
    return 1;
  }
  for (uint i = 0; i < optocount; i++)
  {
    optos.push_back(new MinimalOpto(portnames[i], MinimalOpto::DataFreq::HZ1000));
    cout << "Successfully found optoforce force sensor at " << portnames[i] << endl;
  }


  // Wait a bit
  struct timespec tim;
  tim.tv_sec = 1;
  tim.tv_nsec = 0;
  nanosleep(&tim, NULL);
  
 
  // Reorganize the optoforce sensors

  vector<string> order_names = {"1st", "2nd", "3rd", "4th"};
  for (uint i = 0; i < optocount-1; i++)
  {
    cout << "Please push the " << order_names[i] << " optoforce sensor..." << endl;
    bool found = false;
    while(!found)
    {
      for (uint j = i; j < optocount; j++)
      {
	double this_f[3] = {0, 0, 0}; 
	optos[j]->GetComputedForces(this_f);
       
	if(this_f[2] > threshold_force && this_f[2] < 2 * threshold_force)
	{
	  MinimalOpto* tmp = optos[i];
	  optos[i] = optos[j];
	  optos[j] = tmp;
	  found = true;
	  break;
	}
	
	if(debug)
	{
	  cout << this_f[2] << "\t";
	}
      }
      
      if(debug)
      {
	cout << endl;
      }
	
    }
    cout << "Ok!" << endl;
  }
  
  // Let the sensors settle
  tim.tv_sec = 2;
  tim.tv_nsec = 0;
  nanosleep(&tim, NULL);
  
  
  int count = 0;

  struct timeval  tv;
  double time_in_mill = 0;
  double time_0 = 0;
  int counter = 0;


  // RELATED WITH THE BEEPER
  int k = 0;
  double t0 = 0;
 
  sound_play::SoundClient sc;
  sound_play::SoundClient quiet_sc;
  sound_play::Sound s7 = sc.waveSoundFromPkg("sound_play", "sounds/BACKINGUP.ogg");

  sound_play::Sound s1 = sc.waveSound("/home/julen/catkin_ws/src/audio_common/sound_play/sounds/say-beep.wav");
  
  // RELATED WITH THE KEYBOARD

  // Create keyboard control object
  Keyboard keyboard;

  bool quit = false;
  while (ros::ok() && !quit)
  {
    
    // READ KEYBOARD
    if(keyboard.kbhit())
    {
      char read_ch = keyboard.getch();
      if(read_ch == 'f' || read_ch == 'F')
      {
	T = 0.4;
      }
	else if(read_ch == 's' || read_ch == 'S')
   	{
	T = 1.4;
         }
	else if(read_ch == 'm' || read_ch == 'M')
   	{
	T = 0.8;
         }
        else if(read_ch == 'q' || read_ch == 'Q')
   	{
	quit=true;
         }
    }


    gettimeofday(&tv, NULL);
    time_in_mill = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ; // convert tv_sec & tv_usec to millisecond    
    if(counter++ == 0)
    {
      time_0 = time_in_mill;
    }

    outputFile1 <<  (time_in_mill - time_0) / 1000.0 << " ";
    std::cout << (time_in_mill - time_0) / 1000.0 <<std::endl;
   
    for (uint i = 0; i < optocount; i++)
    {
      double this_f[3] = {0, 0, 0};
      optos[i]->GetComputedForces(this_f);
      f[i] = {this_f[0], this_f[1], this_f[2]};
      outputFile1 <<  f[i][0] << " " <<f[i][1] << " " << f[i][2] <<" ";
    }
    outputFile1 << "\n" << std::endl;

    //std::cout << f[0][0] << " " << f[0][1] << " " << f[0][2] << " " << f[1][0] << " "<< f[1][2] << " " <<
    //std::endl;

    geometry_msgs::Wrench W1;
    geometry_msgs::Wrench W2;
   
   
     W1.force.x = f[0][0];
     W2.force.x = f[1][0];
     W1.force.y = f[0][1];
     W2.force.y = f[1][1];
     W1.force.z = f[0][2];
     W2.force.z = f[1][2];
     
    
    F1.publish(W1);
    F2.publish(W1);

  
    if((time_in_mill - t0)/1000 > T)
    {
     s1.stop();
     k=0;
     }
    if (k==0)
    {
    s1.repeat();
    t0 = time_in_mill;
    k=1;
    }
  
   
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
