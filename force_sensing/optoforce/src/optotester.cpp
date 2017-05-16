
#include <iostream>
#include "MinimalOpto.hpp"

using namespace std;

int main (int argc, char **argv)
{
  MinimalOpto opto("/dev/ttyACM0", MinimalOpto::HZ100);
  
  cout << "Fx\tFy\tFz" << endl;
  while(true)
  {
    double f[3] = {0, 0, 0};
    opto.GetComputedForces(f);
    cout << f[0] << "\t" << f[1] << "\t" << f[2] << endl;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
}