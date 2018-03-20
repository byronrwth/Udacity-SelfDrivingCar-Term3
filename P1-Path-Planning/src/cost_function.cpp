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

    double cost_changelane = 0;
    double cost_inefficiency = 0;
    double cost_collision = 0;
    double cost_buffer = 0;
    double cost_target = 0;

    // try stay on lane
    cost_changelane = ChangeLane();

    // try to reach smeed limit
    cost_inefficiency = Inefficiency();

    cost_collision = Collision();

    cost_buffer = Buffer();
    cost_target = Target();

    cost = cost_changelane + cost_inefficiency + cost_collision + cost_buffer + cost_target;
    // max accerleration

    // max jerk
    if (cost > 0)
    {
        cout << "--------------CostFunction::Compute-------------------" << endl;
        cout << "changelane   --- cost % :" << (cost_changelane/cost/1.0 * 100) << endl;
        cout << "inefficiency --- cost % :" << (cost_inefficiency/cost/1.0 * 100) << endl;
        cout << "collision    --- cost % :" << (cost_collision/cost/1.0 * 100) << endl;
        cout << "buffer       --- cost % :" << (cost_buffer/cost/1.0 * 100) << endl;
        cout << "target       --- cost % :" << (cost_target/cost/1.0 * 100) << endl;

    }


    return cost;
}

double CostFunction::ChangeLane() {
    //Compute cost to change lane, penalizes lane Away fron the leftiest lane (fastest).
    int end_lane = vehicle->trajectory.lane_end;
    int start_lane = vehicle->trajectory.lane_start;
    double cost = 0;
    if (start_lane != end_lane) {
        cost += CHANGELANE;
    }

    return cost;
}

double CostFunction::Inefficiency() {
    //Always, the best efficiency is when the speed is closest to the limit
    double cost = 0;
    double diff = (49.5 - vehicle->update.target_v) / 49.5;
    cost = pow(diff, 2) * INEFFICIENCY;
    return cost;
}




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



double CostFunction::Target(){
  double cost =0;
  if(!vehicle->collider.collision){
    return 0;
  }
  int end_lane = vehicle->trajectory.lane_end;
  int start_lane = vehicle->trajectory.lane_start;
  double diff = (vehicle->collider.target_speed - vehicle->speed)/vehicle->collider.target_speed;
  cost = pow(diff,2) * TARGET;      
  return cost;
}

double CostFunction::Buffer(){
  double cost = 0;

  if(vehicle->collider.closest_approach == 10000){
    return 0;
  }

  double time_steps = abs(vehicle->collider.closest_approach)/(abs(vehicle->speed)*MPH_TO_MS);

  
  if(time_steps > DESIRED_BUFFER){
    return 0;
  }

  double multiplier = 1.0 - pow((time_steps / DESIRED_BUFFER),2);
  cost = multiplier * BUFFER;
  if(vehicle->collider.closest_approach < 0){
    //car in the back
    cost /= 10;
  }       
  return cost;
}
