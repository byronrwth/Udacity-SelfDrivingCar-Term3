#include "cost_function.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const double MS_TO_MPH = 2.23694;
 
const double MPH_TO_MS = 0.44704;

CostFunction::CostFunction(Vehicle *v, vector<vector<double>> s) {
  vehicle = v;
  sensor_fusion = s;
}


double CostFunction::Compute() {

  //compute cost
  // not yet to use
  //double cost = 10000000000;
  double cost = 0;

  // try stay on lane
  cost += ChangeLane();

  // try to reach smeed limit
  cost += Inefficiency();

  //cost += Collision();

  //cost += Buffer();
  //cost += Target();

  // max accerleration

  // max jerk


  return cost;
}

double CostFunction::ChangeLane(){
  //Compute cost to change lane, penalizes lane Away fron the leftiest lane (fastest).
  int end_lane = vehicle->trajectory.lane_end;
  int start_lane = vehicle->trajectory.lane_start;
  double cost = 0;
  if(start_lane != end_lane){
    cost += COMFORT;
  } 
  
  return cost;
}

double CostFunction::Inefficiency() {
  //Always, the best efficiency is when the speed is closest to the limit
  double cost = 0;
  double diff = (49.5 - vehicle->update.target_v) / 49.5;
  cost = pow(diff, 2) * EFFICIENCY;
  return cost;
}

#if 0


double CostFunction::Collision() {
  double cost = 0;
  if (vehicle->collider.collision) {
    double distance =  vehicle->collider.distance;
    //distance divided by the relative speed
    double time_to_collide = abs(vehicle->collider.distance) / (abs(vehicle->speed) * MPH_TO_MS);
    cost = exp(-pow(time_to_collide, 2)) * COLLISION;
    //changing lane
    if (vehicle->trajectory.lane_end != vehicle->trajectory.lane_start) {
      if (time_to_collide > DESIRED_BUFFER) {
        //safe to change lane
        cost /= 10;
      }
    }
  }
  return cost;
}


#endif