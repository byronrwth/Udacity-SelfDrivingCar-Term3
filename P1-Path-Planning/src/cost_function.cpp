#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

CostFunction::CostFunction(Vehicle *v, vector<vector<double>> s){
  vehicle = v;
  sensor_fusion = s;
}


double CostFunction::Compute(){
  
  //compute cost
  double cost = 0;
  cost += ChangeLane();  
  cost += Inefficiency();
  cost += Collision();
  cost += Buffer();
  cost += Target();
  
  // max accerleration

  // max jerk


  return cost;
}


double CostFunction::Inefficiency(){
  //Always, the best efficiency is when the speed is closest to the limit
  double cost = 0;
  double diff = (49.5 - vehicle->update.target_v)/49.5;
  cost = pow(diff,2) * EFFICIENCY;  
  return cost;
}