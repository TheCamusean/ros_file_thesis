
#include "MinimalOpto.hpp"
#include <cmath>

optoexception optoex;

bool MinimalOpto::Flush()
{
  if(tcflush(com_port_handle_, TCIOFLUSH) == -1)
  {
    return false;
  }  
  return true;
}

bool MinimalOpto::OpenComPort()
{
  com_port_handle_ = open(com_port_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (com_port_handle_ == -1)
  {
    DEBUGACT(printf("\nProblem opening the port %s!\n", com_port_path_.c_str()))
    return false;
  }
  DEBUGACT(printf("Opened! COM port = %i\n", com_port_handle_))
 
  struct termios options;
  tcgetattr(com_port_handle_, &options);
  
  cfsetospeed(&options, baud_rate_); //set the baud rate 
  cfsetispeed(&options, baud_rate_); //set the baud rate 

  options.c_cflag &= ~CSIZE;  // mask the character size bits
  options.c_cflag |= CS8;     // 8 data bits
  options.c_cflag &= ~CSTOPB; // set the number of stop bits to 1
  options.c_cflag &=~PARENB;  // set parity to None
  //set for non-canonical (raw processing, no echo, etc.)
  options.c_iflag = IGNPAR; // ignore parity check
  options.c_oflag = 0;      // raw output
  options.c_lflag = 0;      // raw input
  //Time-Outs -- won't work with NDELAY option in the call to open
  options.c_cc[VMIN] = 0;  // block reading until RX x characers. If x = 0, it is non-blocking.
  options.c_cc[VTIME] = 1; // Inter-Character Timer -- i.e. timeout= x*.1 s
  //Set local mode and enable the receiver
  options.c_cflag |= (CLOCAL | CREAD);

  Flush();
  int status = tcsetattr(com_port_handle_, TCSANOW, &options);

  if (status != 0)
  {
    DEBUGACT(printf("\nProblem setting port options..!\n"))
    return false;
  }

  Flush();
  
  return true;
}

void MinimalOpto::CloseComPort()
{
  close(com_port_handle_);
}

bool MinimalOpto::ReadPort()
{
  struct termios initial_settings, new_settings;
  Byte response[READBUFFSIZE] = {0};
  
  //get current terminal settings 
  tcgetattr(0, &initial_settings);
  new_settings = initial_settings;
  new_settings.c_lflag &= ~ICANON;
  new_settings.c_cc[VMIN] = 0;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_settings);

  int size = 0;
  do
  {
    size = read(com_port_handle_, &response[0], READBUFFSIZE);
    if(size > 0)
    {
      content_.Add(response, size);
      DEBUGACT(for(int i = 0; i < size; i++) printf("%2x ", response[i]))
    }
  } while(size > 0);
  
  tcsetattr(0, TCSANOW, &initial_settings);
  
  return true;
}

bool MinimalOpto::SendCommand(std::string CommandString)
{
  unsigned char * hexs;
  
  if(CommandString.size() % 2 != 0)
  {
    DEBUGACT(printf("\nCommand size is odd..!"))
    return false;
  }

  //each hex char is only 1 byte from the two entered
  int number_hex_chars = CommandString.size() / 2;
  hexs = (unsigned char*) malloc(number_hex_chars * sizeof(unsigned char));

  //process each 2-char set from an ascii representation of the hexadecimal command byte to the associated byte
  for(int i = 0; i < number_hex_chars; i++)
  {
    //if(sscanf(&CommandString[i * 2], "%2x", &hexs[i]) < 1)
    if(sscanf(&CommandString[i * 2], "%2c", &hexs[i]) < 1)
    {
      DEBUGACT(printf("\nProblem with sscanf..!"))
      return false;
    }
  }
  
  write(com_port_handle_, hexs, number_hex_chars); 
  free(hexs); 
  return true;
}

MinimalOpto::MinimalOpto(std::string PortName, DataFreq Freq)
{ 
  updatecount = 0;
  calibrated_ = 0;
  calibration_f_[0] = 0; calibration_f_[1] = 0; calibration_f_[2] = 0;
  computed_f_[0] = 0; computed_f_[1] = 0; computed_f_[2] = 0;
  last_computed_f_[0] = 0; last_computed_f_[1] = 0; last_computed_f_[2] = 0;
  
  halt_ = false;
  com_port_path_ = PortName;  
  baud_rate_ = B115200;

  bool opened = OpenComPort();
  if(!opened)
  {
    cerr << "---!!!---" << PortName << endl;
    throw optoex;
  }

  std::string command_configure_byte;
  switch(Freq)
  {
    case DataFreq::HZ30:   command_configure_byte = "AA0032032104000102"; break; // 170 0 50 3 33 4 255 2 3   --> AA 00 32 03 21 04 FF 02 03
    case DataFreq::HZ100:  command_configure_byte = "AA0032030A040000EB"; break; // 170 0 50 3 10 4 255 1 236 --> AA 00 32 03 0A 04 FF 01 EC
    case DataFreq::HZ333:  command_configure_byte = "AA00320303040000E4"; break; // 170 0 50 3 3 4 255 1 229  --> AA 00 32 03 03 04 FF 01 E5
    case DataFreq::HZ1000: command_configure_byte = "AA00320301040000E2"; break; // 170 0 50 3 1 4 255 1 227  --> AA 00 32 03 01 04 FF 01 E3
  }

  SendCommand(command_configure_byte);
  
  continuous_update_thread_ = std::thread(&MinimalOpto::ContinuouslyUpdateValues, this); 
}

MinimalOpto::~MinimalOpto()
{
  halt_ = true;
  CloseComPort();
  continuous_update_thread_.join();
}

bool MinimalOpto::UpdateValues()
{
  ReadPort();
  
  int data_ind = FindValidSignature();
  if(data_ind == -1)
  {
    return false;
  }
      
  union { char c[2]; int i; } value_converter;
    
  val_mutex_.lock();

  value_converter.c[1] = content_[data_ind +  0]; value_converter.c[0] = content_[data_ind +  1]; f_[0] = value_converter.i;
  value_converter.c[1] = content_[data_ind +  2]; value_converter.c[0] = content_[data_ind +  3]; f_[1] = value_converter.i;
  value_converter.c[1] = content_[data_ind +  4]; value_converter.c[0] = content_[data_ind +  5]; f_[2] = value_converter.i;


  vector<double> sensitivity = {8020, 8174, 15923}; // counts @ nominal capacity
  vector<double> nc = {60, 60, 450}; // nominal capacity in Newtons

  for (uint i = 0; i < 3; i++)
  {
    if(f_[i] > 32767)
    {
      f_[i] -= 65534;
    }
    
    computed_f_[i] = (double)f_[i] / sensitivity[i] * nc[i] - calibration_f_[i]; // (double)f_[i]; // / sensitivity[i] * nc[i]; //- calibration_f_[i];
  }
  
  calibrated_++;
  
  if(calibrated_ == 1)
  {
    calibration_f_[0] = computed_f_[0];
    calibration_f_[1] = computed_f_[1];
    calibration_f_[2] = computed_f_[2];
    
    computed_f_[0] = 0;
    computed_f_[1] = 0;
    computed_f_[2] = 0;
    
    cout << "Optoforce sensor on " << com_port_path_ << " is now calibrated" << endl;
  }

  last_computed_f_[0] = computed_f_[0];
  last_computed_f_[1] = computed_f_[1];
  last_computed_f_[2] = computed_f_[2];

  val_mutex_.unlock();
  
  updatecount++;

  // WARNING: DIRTY HACK HERE, PLEASE DO NOT HATE MOS :(
  // THIS IS TO SOLVE THE ISSUE OF BLOCKING THE RT TASK FOR ONCILLA CLOSED LOOP CONTROL
  // NO TIME, HAVE TO WRITE THE THESIS!
  struct timespec tim;
  tim.tv_sec = 0;
  tim.tv_nsec = 500000;
  nanosleep(&tim, NULL);

  //if(updatecount % 20000 == 0)
  //  cout << "[[MinimalOpto]] " << com_port_path_ << " is alive!" << endl;

  return true;
}

int MinimalOpto::FindValidSignature()
{
  // 37 44 b7 ff d5  0 24 74 2b 6d 11 7f b5 80 75 82 42 86 cd 74 10 74 1f 74 3b 74 44  a 8a THIS IS OLD
  // 170 7 8 10 C C S S X X Y Y Z Z K K
  int ind = -1;
  int min_data_length = 16; //29;
  for(int i = STOREBUFFSIZE - min_data_length; i >= 0; i--)
  {
    if(content_[i] == 170 && content_[i + 1] == 7 && content_[i + 2] == 8 && content_[i + 3] == 10)
    {
      ind = i + 8;
      break;
    }
  } 
  return ind;
}

void MinimalOpto::GetValues(int & temperature, int f[], int raw_s[], int comp_s[])
{
  val_mutex_.lock();
  
  temperature = temperature_;
  
  f[0] = f_[0]; 
  f[1] = f_[1]; 
  f[2] = f_[2]; 
  
  raw_s[0] = raw_s_[0];
  raw_s[1] = raw_s_[1];
  raw_s[2] = raw_s_[2];
  raw_s[3] = raw_s_[3];

  comp_s[0] = comp_s_[0];
  comp_s[1] = comp_s_[1];
  comp_s[2] = comp_s_[2];
  comp_s[3] = comp_s_[3];

  val_mutex_.unlock();

}

void MinimalOpto::GetComputedForces(double computed_f[])
{
  
  val_mutex_.lock();

  computed_f[0] = computed_f_[0];
  computed_f[1] = computed_f_[1];
  computed_f[2] = computed_f_[2];

  last_computed_f_[0] = 0;
  last_computed_f_[1] = 0;
  last_computed_f_[2] = 0;

  val_mutex_.unlock();  
  
}

void MinimalOpto::ContinuouslyUpdateValues()
{
  while(!halt_)
  {
    UpdateValues();
  }
}
