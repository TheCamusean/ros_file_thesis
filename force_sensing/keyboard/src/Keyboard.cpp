
#include "Keyboard.hpp"

#include <iostream>

Keyboard::Keyboard()
{
  tcgetattr(0,&initial_settings);
  new_settings = initial_settings;
  new_settings.c_lflag &= ~ICANON;
  new_settings.c_lflag &= ~ECHO;
  new_settings.c_lflag &= ~ISIG;
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_settings);
  peek_character = -1;
}

Keyboard::~Keyboard()
{
  tcsetattr(0, TCSANOW, &initial_settings);
}

int Keyboard::kbhit()
{
  unsigned char ch;
  int nread;

  if (peek_character != -1) 
    return 1;
  
  //new_settings.c_cc[VMIN]=0;
  //tcsetattr(0, TCSANOW, &new_settings);
  //nread = read(0,&ch,1);
  fd_set set;
  FD_ZERO(&set);
  FD_SET(0, &set);
  timeval tv = { 0, 0 };
  int ret = select(1, &set, NULL, NULL, &tv);
  if (ret == 0) {
    return 0;
  } else if (ret == 1) {
    return 1;
  } else {
    perror("select()");
  }
  
  //new_settings.c_cc[VMIN]=1;
  //tcsetattr(0, TCSANOW, &new_settings);
/*q
  if (nread == 1) 
  {
      peek_character = ch;
      std::cout << "buffer was = " << ch << std::endl;
      return 1;
  }
  return 0;*/
}

int Keyboard::getch()
{
  char ch;

  if (peek_character != -1) 
  {
      ch = peek_character;
      peek_character = -1;
  } 
  else if(read(0,&ch,1)) 
  {
  }

  return ch;
}
