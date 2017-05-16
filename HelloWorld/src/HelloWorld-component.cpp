#include "HelloWorld-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

HelloWorld::HelloWorld(std::string const& name) : TaskContext(name){
  std::cout << "HelloWorld constructed !" <<std::endl;
}

bool HelloWorld::configureHook(){
  std::cout << "HelloWorld configured !" <<std::endl;
  return true;
}

bool HelloWorld::startHook(){
  std::cout << "HelloWorld started !" <<std::endl;
  return true;
}

void HelloWorld::updateHook(){
  std::cout << "HelloWorld executes updateHook !" <<std::endl;
}

void HelloWorld::stopHook() {
  std::cout << "HelloWorld executes stopping !" <<std::endl;
}

void HelloWorld::cleanupHook() {
  std::cout << "HelloWorld cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HelloWorld)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(HelloWorld)
