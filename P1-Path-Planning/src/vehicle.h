#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

enum class States{ KL, LCL, LCR, PLCL, PLCR };

class Vehicle {
 public:


  States state = States::KL;

  struct update {
    double ref_v = 0;
    double target_v = 49.50;
    int lane = 0;
  } update;

  struct collider {
    bool collision = false;
    double distance = 0;
    double changelane_gap = 1000;
    double target_speed = 0;
  } collider;

  struct trajectory {
    int lane_start = 0;
    int lane_end = 0;
    double target_speed = 0;
  } trajectory;

  double ref_speed = 0;
  int ref_lane = 0;

  double x = 0;
  double y = 0;
  double s = 0;
  double d = 0;
  double yaw = 0;
  double speed = 0;

  double delta_time = 0;

  // only to calc monitor max speed, acceleration, jerk !
  double speed_mps = 0.0; // in m/s mps
  double prev_speed_mps = 0.0; // in m/s mps 
  double prev_accel_mpss = 0.0;  // in m/s/s 
  double accel_mpss = 0.0;      // in m/s/s 
  double jerk_mpsss = 0.0;    // in m/s/s/s 

  /**
  * Constructor
  */
  //Vehicle();
  //Vehicle(int lane, float s, float v, float a, string state="CS")
  Vehicle(int lane, double target_speed);

  void Update(double ax, double ay, double as, double ad, double ayaw, double aspeed, int lane, double target_speed, double delta);

  void NextState(vector<vector<double>> sensor);

  /**
  * Destructor
  */
  //virtual ~Vehicle();

 private:
  void _realise_state(States state, vector<vector<double>> sensor_fusion);
  void _reset_data();

};

#endif