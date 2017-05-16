// use -lpthread -std=c++0x in compilation

#ifndef _MINIMALOPTO_H_
#define _MINIMALOPTO_H_


#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>  // File control definitions
#include <errno.h>  // Error number definitions
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <iostream>

#define DEBUG false
#define READBUFFSIZE 4096
#define STOREBUFFSIZE 10258

#define DEBUGACT(X) if(DEBUG) { X; }

using namespace std;

#define ABSMAX(A,B) ((abs(A)>abs(B))?A:B)

class optoexception: public exception
{
  virtual const char* what() const throw()
  {
    return "Opto not found.";
  }
};

class ConveyorBelt
{
private:
  typedef unsigned char Byte;
  Byte vals_[STOREBUFFSIZE];
  int index_;

public:
  ConveyorBelt() { index_ = 0; }
  ~ConveyorBelt() {}
  void Add(Byte Item)
  {
    vals_[index_] = Item;
    index_ = (index_ + 1) % STOREBUFFSIZE;
  }
  void Add(Byte * Items, int Size)
  {
    for(int i = 0; i < Size; i++)
    {
      Add(Items[i]);
    }
  }
  Byte& operator[] (const int ReqIndex)
  {
    return vals_[(index_ + ReqIndex) % STOREBUFFSIZE];
  }
};

class MinimalOpto
{
private:
  typedef unsigned char Byte;
  ConveyorBelt content_;
  bool halt_;
  int baud_rate_;
  int com_port_handle_;

  bool last_failed_;
  int temperature_;
  int f_[3];
  int raw_s_[4];
  int comp_s_[4];
  int calibrated_;
  double calibration_f_[3];
  double computed_f_[3];
  double last_computed_f_[3];
  std::mutex val_mutex_;
  std::thread continuous_update_thread_ ;
  int updatecount;

  bool Flush();
  bool OpenComPort();
  void CloseComPort();
  bool ReadPort();
  bool SendCommand(std::string CommandString);
  void ContinuouslyUpdateValues();
  int FindValidSignature();
  bool UpdateValues();
  std::string com_port_path_;
public:
  enum DataFreq {HZ30, HZ100, HZ333, HZ1000};
  MinimalOpto(std::string PortName, DataFreq Freq);
  ~MinimalOpto();
  void GetValues(int & temperature, int f[], int raw_s[], int comp_s[]);
  void GetComputedForces(double computed_f[]);
};

#endif
