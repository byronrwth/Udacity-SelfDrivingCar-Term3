#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  States state = KL;


  /**
  * Constructor
  */
    Vehicle();
    //Vehicle(int lane, float s, float v, float a, string state="CS")
    Vehicle(int lane, double target_speed);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  
}