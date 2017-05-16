#ifndef OROCOS_HELLOWORLD_COMPONENT_HPP
#define OROCOS_HELLOWORLD_COMPONENT_HPP

#include <rtt/RTT.hpp>

class HelloWorld : public RTT::TaskContext{
  public:
    HelloWorld(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
