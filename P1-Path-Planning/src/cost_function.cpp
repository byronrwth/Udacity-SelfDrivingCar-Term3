#include "cost_function.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const double MS_TO_MPH = 2.23694;

const double MPH_TO_MS = 0.44704;

bool debug_CostFunction = false;

CostFunction::CostFunction(Vehicle *v, vector<vector<double>> s) {
    vehicle = v;
    sensor_fusion = s;
}


double CostFunction::Compute() {

    //compute cost
    // not yet to use
    //double cost = 10000000000;
    double cost = 0;

    double cost_changelane_comfort = 0;
    double cost_inefficiency = 0;
    double cost_collision = 0;
    double cost_changelane_danger = 0;
    double cost_target = 0;

    // try stay on lane
    cost_changelane_comfort = changelane_comfort();

    // try to reach smeed limit
    cost_inefficiency = Inefficiency();

    cost_collision = Collision();

    cost_changelane_danger = changelane_danger();
    cost_target = Target();

    cost = cost_changelane_comfort + cost_inefficiency + cost_collision + cost_changelane_danger + cost_target;

    // max accerleration < 10 m /s/s

    // max jerk < < 10 m /s/s/s

    if ( debug_CostFunction == true && cost > 0)
    {
        //cout << "--------------CostFunction::Compute-------------------" << endl;
        //cout << "changelane       x (" << CHANGELANE_COMFORT   << ") --- cost % :" << (cost_changelane/cost/1.0 * 100) << endl;
        //cout << "inefficiency     x (" << INEFFICIENCY << ") --- cost % :" << (cost_inefficiency/cost/1.0 * 100) << endl;
        //cout << "collision        x (" << COLLISION    << ") --- cost % :" << (cost_collision/cost/1.0 * 100) << endl;
        //cout << "buffer           x (" << CHANGELANE_DANGER       << ") --- cost % :" << (cost_buffer/cost/1.0 * 100) << endl;
        //cout << "target           x (" << TARGET       << ") --- cost % :" << (cost_target/cost/1.0 * 100) << endl;
//

        cout << "\n" << endl;
        cout << "---------------------CostFunction::Compute in % -------------------" << endl;
        cout << "    collision  |  changelane_danger   |   changelane_comfort  |   inefficiency  |    target    "   << endl;
        
        cout << "    x(" << COLLISION  << ")   |  x(" << CHANGELANE_DANGER << ")     |   x(" << CHANGELANE_COMFORT << ")      |   x(" << INEFFICIENCY  << ")   |    x(" << TARGET  << ")     "   << endl;

        cout << "    " << (int)floor(cost_collision/cost/1.0 * 100) << "      |  " << (int)floor(cost_changelane_danger/cost/1.0 * 100) << "        |   " << (int)floor(cost_changelane_comfort/cost/1.0 * 100) << "       |   " << (int)floor(cost_inefficiency/cost/1.0 * 100) << "       |    " << (int)floor(cost_target/cost/1.0 * 100) << "    " << endl;
        cout << "\n" << endl;

    }


    return cost;
}

/*
The change lane cost function adds a "comfort" constant penalty if the vehicle decides to change lane 
*/
double CostFunction::changelane_comfort() {
    //Compute cost to change lane, penalizes lane Away fron the leftiest lane (fastest).
    int end_lane = vehicle->trajectory.lane_end;
    int start_lane = vehicle->trajectory.lane_start;
    double cost = 0;
    if (start_lane != end_lane) {
        cost += CHANGELANE_COMFORT;
    }

    return cost;
}

/*
This function evaluates the vehicle speed defined in this state in relation to the maximum speed allowed. States with speeds closer to the maximum speed are more efficient (lower cost), in contrast, states with lower speeds are less efficient (higher cost).
*/
double CostFunction::Inefficiency() {
    //Always, the best efficiency is when the speed is closest to the limit
    double cost = 0;
    double diff = (49.5 - vehicle->update.target_v) / 49.5;
    cost = pow(diff, 2) * INEFFICIENCY;
    return cost;
}


/*
The collision cost is the most important function. It strongly penalizes the states which the risk of collision is more imminent. However, in order to force the car to escape from heavy traffic, the collision cost is smaller whenever is safe to change lane. We found out that it helps the car to find a more appropriate situation, instead of just following the car ahead until it opens a passageway 
*/
double CostFunction::Collision() {
    double cost = 0;
    if (vehicle->collider.collision) {
        double distance =  vehicle->collider.distance;
        //distance divided by the relative speed
        double time_to_collide = abs(vehicle->collider.distance) / (abs(vehicle->speed) * MPH_TO_MS);
        cost = exp(-pow(time_to_collide, 2)) * COLLISION;
        //changing lane
        if (vehicle->trajectory.lane_end != vehicle->trajectory.lane_start) {
            if (time_to_collide > CHANGELANE_DURATION) {
                //cout << "-----------------Collision()::time_to_collide:" << time_to_collide << "-----------------"<< endl;
                // if possible to change lane, then reduce collision cost at same level as change lane danger, otherwise collision cost can 10 times larger, i.e. always occupy 90% of sum(costs), then vehicle won't dare to change lane !
                cost /= 10;
                
            }
        }
    }
    return cost;
}

/*
The target cost evaluates the speed comparison between the ego car and the vehicle in front (possible collision). If all lanes are blocked (possible collision), this function helps the car to choose a lane which the speed of the vehicle in question matches closer to the ego car
*/
double CostFunction::Target(){
  double cost =0;

  // no collision, --> no car in ahead to affect ego car speed, so cost=0
  if(!vehicle->collider.collision){
    return 0;
  }
  //int end_lane = vehicle->trajectory.lane_end;
  //int start_lane = vehicle->trajectory.lane_start;

  // im mph, collider.target_speed can be set by neighboring lane cars !
  double diff = (vehicle->collider.target_speed - vehicle->speed)/vehicle->collider.target_speed;
  cost = pow(diff,2) * TARGET;      
  return cost;
}

/*
The buffer cost function computes how long it has to the vehicle in front. It is computed by dividing the distance from the vehicle in front by the current speed of the ego car. Note that the cost is smaller if the vehicle in question is behind. It helps the ego car to choose a lane with no traffic in the front 
*/
double CostFunction::changelane_danger(){
  double cost = 0;

  if(vehicle->collider.changelane_gap == 10000){
    return 0;
  }

  double time_steps = abs(vehicle->collider.changelane_gap)/(abs(vehicle->speed)*MPH_TO_MS);

  
  if(time_steps > CHANGELANE_DURATION){
    return 0;
  }

  double multiplier = 1.0 - pow((time_steps / CHANGELANE_DURATION),2);
  //cout << "time_steps: " << time_steps <<  " ,pow((time_steps / CHANGELANE_DURATION),2):" <<pow((time_steps / CHANGELANE_DURATION),2) << " , multiplier:" << multiplier << endl;
  cost = multiplier * CHANGELANE_DANGER;
  if(vehicle->collider.changelane_gap < 0){
    //car in the back
    cost /= 10;
  }       
  return cost;
}
