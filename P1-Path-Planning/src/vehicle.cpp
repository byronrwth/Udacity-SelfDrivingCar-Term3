#include "vehicle.h"

#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>

#include "cost_function.h"

/**
 * Initializes Vehicle
 */


/*
Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;

}
*/
const double MS_TO_MPH = 2.23694;

const double MPH_TO_MS = 0.44704;

bool debug = false;



Vehicle:: Vehicle(int lane, double target_speed) {
    ref_speed = target_speed;
    ref_lane = lane;
}

void Vehicle::Update(double ax, double ay, double as, double ad, double ayaw, double aspeed, int lane, double target_speed, double delta) {
    //update raw data
    x = ax;
    y = ay;
    s = as;
    d = ad;
    yaw = ayaw;
    speed = aspeed;


    delta_time = delta;

    ref_speed = target_speed;

    cout << "--------------Update-------------------" << endl;
    cout << "speed --- target :" << endl;
    cout << " " << speed
         << " --- "
         << " " << ref_speed  << endl;



    ref_lane = lane;

    //clean data
    _reset_data();
}

void Vehicle::_reset_data() {
    //clean data

    //reset trajectory
    trajectory.lane_start = ref_lane;

    trajectory.lane_end = ref_lane;


    trajectory.target_speed = ref_speed;
    update.lane = ref_lane;

    //cout << "vehicle: _reset_data(): trajectory.target_speed= "  << trajectory.target_speed << endl;
    cout << "--------------_reset_data trajectory-------------------" << endl;
    cout << "lane_start --- lane_end --- ref_lane:" << endl;
    cout << "   " << trajectory.lane_start
         << "  ---  "
         << trajectory.lane_end
         << "  ---  "
         << ref_lane << endl;




    //reset update
    update.ref_v = ref_speed;




    update.target_v = 49.50; // in mph


    cout << "--------------_reset_data-------------------" << endl;
    cout << "--- update.ref_v(ref_speed) --- :" << endl;
    cout << "    " << update.ref_v<< endl;



    collider.collision = false;

    //collider.distance = 10000;
    //collider.closest_approach = 10000;
    //collider.target_speed = 0;
}


void Vehicle::NextState(vector<vector<double>> sensor) {

    States current_state = state;

    cout << "--------------NextState-------------------" << endl;
    cout << "current_state --- state:" << endl;
    cout << "  " << current_state
         << " --- "
         << "  " << state << endl;

    vector<States> states;
    //select reachable states

    // add a new element to end, len +1
    states.push_back(KL);

    cout << "add:  KL --- new size:--- "<<states.size()<< endl;


    if (state == PLCL) {
        states.push_back(LCL);
        states.push_back(PLCL);
        cout << "add: LCL, PLCL --- new size:--- "<<states.size()<< endl;

    } 
    else if (state == PLCR) {
        states.push_back(LCR);
        states.push_back(PLCR);
        cout << "add: LCR, PLCR --- new size:--- "<<states.size()<< endl;

    } 
    else {

        cout << "ref_lane:  " << ref_lane << " --- update.lane: ---"<<update.lane<< endl;


        cout << "ref_speed: --- speed: " << endl;
        cout << "  " << ref_speed
             << " --- "
             << "  " << speed << endl;

        if (ref_lane != 0) {
            //check if lane change is over before LCL again
            if (d < (2 + 4 * (ref_lane) + 2) && d > (2 + 4 * (ref_lane) - 2)
                    && speed > 20) {
                //inside lane
                states.push_back(PLCL);
                cout << "add:  PLCL --- new size():--- "<<states.size()<< endl;

            }
        }
        if (ref_lane != 2) {
            //check if lane change is over before LCR again
            if (d < (2 + 4 * (ref_lane) + 2) && d > (2 + 4 * (ref_lane) - 2)
                    && speed > 20) {
                states.push_back(PLCR);
                cout << "add:  PLCR --- new size():--- "<<states.size()<< endl;

            }
        }
    }


    States min_state = KL;
    double  min_cost = 10000000;

    //compute cost of all reachable states
    for (int i = 0; i < states.size(); i++) {
        States n_state = states[i];

        //prepare state
        cout << "--------------prepare state-------------------" << endl;
        cout << "if states["<<i<<"] --- :"<<n_state<< endl;
        cout << " " << speed
             << " --- "
             << " " << ref_speed  << endl;

        _reset_data();
        _realise_state(n_state, sensor);

        CostFunction cost = CostFunction(this, sensor);

        double value = cost.Compute();
        cout << " cost --- :"<<value<< endl;

        if (value < min_cost) {
            min_state = n_state;
            min_cost = value;
            cout << "min cost  --- min_state:"<< endl;
            
            cout << " " << min_cost
                 << " --- "
                 << " " << min_state  << endl;
        }

    }

    //update state
    state = min_state;
    _reset_data();
    _realise_state(state, sensor);


    //update speed
    CostFunction cost = CostFunction(this, sensor); // this  and state  ???

    //float v = cost.Compute();
    double new_value = cost.Compute();

    if (
        //!collider.collision && 
        ref_speed < update.target_v && ref_speed < 49.5) {
        update.ref_v += 0.224;
        //cout << "vehicle: NextState(): increase update.ref_v= "  << update.ref_v << endl;

    } else if (ref_speed > update.target_v && ref_speed > 0) {
        update.ref_v -= 0.224;
        //cout << "vehicle: NextState(): minus update.ref_v= "  << update.ref_v << endl;
    }

    cout << "--------------NextState-------------------" << endl;
    cout << "update.ref_v - ref_speed - update.target_v:" << endl;
    cout << "    " << update.ref_v
         << " --- "
         << "  " << ref_speed
         << " --- "
         << "  " << update.target_v << endl;

    // 
    std::cout << "NEW STATE " << state << " with cost " << min_cost << "\n";

}

void Vehicle::_realise_state(States astate, vector<vector<double>> sensor_fusion) {


    state = astate;
    cout << "--------------_realise_state state-------------------" << endl;
    cout << "   " << state << "   "  << endl;


    switch (state) {
    case KL: {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane;
        update.lane = ref_lane;
        break;
    }

    case PLCL: {
        //same lane
        trajectory.lane_start = ref_lane;

        trajectory.lane_end = ref_lane - 1;
        update.lane = ref_lane;
        break;
    }
    case LCL: {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane - 1;
        update.lane = ref_lane - 1;
        break;
    }
    case PLCR: {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane + 1;
        update.lane = ref_lane;
        break;
    }
    case LCR: {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane + 1;
        update.lane = ref_lane + 1;
        break;
    }

    cout << "state - ref_lane - trajectory.lane_start - trajectory.lane_end - update.lane:" << endl;
    cout << "  " << state
         << " -- "
         << "  " << ref_lane
         << " -- "
         << "  " << trajectory.lane_start
         << " -- "
         << "  " << trajectory.lane_end
         << " -- "
         << "  " << update.lane << endl;


    default:
        std::cout << "STATE ERROR\n";
    }

    //check lane
    if (trajectory.lane_end < 0) {
        trajectory.lane_end = 0;
    } else if ( trajectory.lane_end > 2) {
        trajectory.lane_end = 2;
    }

    if (trajectory.lane_start < 0) {
        trajectory.lane_start = 0;
    } else if ( trajectory.lane_start > 2) {
        trajectory.lane_start = 2;
    }

    if (update.lane < 0) {
        update.lane = 0;
    } else if (update.lane > 2) {
        update.lane = 2;
    }

    double target_speed_front = 0;
    double target_distance_front = 10000;
    double target_speed_lane_front = 0;
    double target_distance_lane_front = 10000;
    double target_speed_lane_back = 0;
    double target_distance_lane_back = -10000;

    double dist_to_collision  = 10000;
    double check_speed = 0;

    //compute collision on start and end lane
    for (int i = 0; i < sensor_fusion.size(); i++) {
        //car is in my lane
        float checkcar_d = sensor_fusion[i][6];

        // trajectory.lane_start always == reflane, my lane
        //Safety check for speed of car in front
        if ((checkcar_d < (2 + 4 * (trajectory.lane_start) + 2) && checkcar_d > (2 + 4 * (trajectory.lane_start) - 2))) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            //delta_time = prev_size * 0.02
            check_car_s += ((double) delta_time * check_speed);

            //check s values greater than mine and s gap
            dist_to_collision = (check_car_s - s);

            if ((check_car_s >= s) && (dist_to_collision < 30)) {

                cout << "*****************************************************" << endl;
                cout << "------------------detect collision-------------------" << endl;
                cout << "distance (0 ~ 30) - check car speed " << endl;
                cout << "  " << dist_to_collision
                     << " --- "
                     << "  " << check_speed
                     << " --- " << endl;


                //debug = true;

                if (target_distance_front > dist_to_collision) {

                    // safety speed? what use ?

                    // why -2 ?
                    //target_speed_front = check_speed * MS_TO_MPH - 2;
                    target_speed_front = check_speed * MS_TO_MPH ;

                    //cout << " ------target_speed_front in mph: " << target_speed_front << endl;

                    target_distance_front = dist_to_collision;// e.g. from 10000 to 30m

                    cout << " check car speed x 2.23 ( in mph)" << endl;
                    cout << "  " << target_speed_front
                         << endl;

                }
            }
        }

        // for KL don't need consider trajectory.lane_end == ref_lane
        // for PLCL, LCL, PLCR, LCR, trajectory.lane_end = trajectory.lane_start +/- 1

#if 0
        // sensed a car on my next move lane
        if (checkcar_d < (2 + 4 * (trajectory.lane_end) + 2) && checkcar_d > (2 + 4 * (trajectory.lane_end) - 2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double) delta_time * check_speed);

            //check s values greater than mine and s gap
            double dist_to_collision = (check_car_s - s);

            if ((trajectory.lane_end != trajectory.lane_start 
                && (abs(dist_to_collision) < 30))
                    || ((check_car_s >= s) && (dist_to_collision < 30))) {

                if (collider.distance > abs(dist_to_collision)) {
                    collider.distance = abs(dist_to_collision);
                    collider.collision = true;
                    collider.closest_approach = abs(dist_to_collision);
                    collider.target_speed = check_speed * MS_TO_MPH;

                    if (abs(dist_to_collision) > 30) {
                        //change targe speed
                        if (check_car_s >= s) {
                            //car in front
                            update.target_v = check_speed * MS_TO_MPH - 2;
                            if (target_distance_lane_front > dist_to_collision) {
                                target_speed_lane_front = check_speed * MS_TO_MPH;
                                target_distance_lane_front = dist_to_collision;
                            }
                        } else {
                            //car in back
                            update.target_v = check_speed * MS_TO_MPH + 2;
                            if (target_distance_lane_back < dist_to_collision) {
                                target_speed_lane_back = check_speed * MS_TO_MPH;
                                target_distance_lane_back = dist_to_collision;
                            }
                        }
                    }
                }
            } 
            else if (!collider.collision
                       && collider.closest_approach > dist_to_collision) {
                collider.closest_approach = dist_to_collision;
                collider.target_speed = check_speed * MS_TO_MPH;
            }
        }// if sensor a car on other lane
#endif

    }// end for

#if 0
    //Safety Speed check
    if (state == PLCL || state == PLCR) {
        //safety speed adjust
        if (target_speed_lane_back != 0 && update.target_v < target_speed_lane_back) {
            update.target_v = target_speed_lane_back;
        }
        if (target_speed_lane_front != 0 && update.target_v > target_speed_lane_front) {
            update.target_v = target_speed_lane_front;
        }
    }

#endif

    if (target_speed_front != 0 && update.target_v > target_speed_front) {
        //update.target_v = target_speed_front - 2;
        update.target_v = target_speed_front - 1;
        cout << " ------reduce 1 mph tobe my target speed : " << endl ;
        cout << "  " << update.target_v
             << endl;
        cout << "-----------------------------------------------------" << endl;
        cout << "*****************************************************" << endl;
    }




}







