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

bool debug  = false;

bool debug_distance = false;

bool debug_lane = false;

bool debug_speed = false;

bool debug_jerk = false;

bool debug_state = false;

bool debug_cost = false;

void print_state(States state){

  if (state == States::KL) {
    cout << "              KL               ";
  } else if (state == States::LCL) {
    cout << "              LCL               ";
  } else if (state == States::LCR) {
    cout << "              LCR               ";
  } else if (state == States::PLCL) {
    cout << "              PLCL               ";
  } else if (state == States::PLCR) {
    cout << "              PCLR               ";
  } else {
    cout << "UNDEFINED";
  }
}

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
    speed = aspeed;  // in mph

    delta_time = delta;
    //std::cout << "********************** delta_time : " << delta_time << "*******************************" << std::endl;
    speed_mps = speed * MPH_TO_MS ;

    accel_mpss = abs(speed_mps - prev_speed_mps ) / delta_time /1.0 ; // m/s/s
    jerk_mpsss = abs(accel_mpss - prev_accel_mpss) / delta_time /1.0 ;  // m/s/s/s



    if (debug_jerk) {
        
        cout << "             accel:" << accel_mpss << "        jerk:" << jerk_mpsss << "             " << endl;

    }

    if ( accel_mpss > 10 || jerk_mpsss > 10 ) {
        std::cout << "\n" << std::endl;
        std::cout << "********************** exceed !! *******************************" << std::endl;
        std::cout << "********************************************************" << std::endl;
        std::cout << "***************** accel :" << accel_mpss << "      jerk: " << jerk_mpsss << "***************" <<std::endl;
        std::cout << "********************************************************" << std::endl;
        std::cout << "********************************************************" << std::endl;
        std::cout << "\n" << std::endl;

        
    }


    // set 9.5 m/s/s as secure buffer, now reduce speed to avoid exceed Max accel
    if ( accel_mpss > 9.5 ) {
        if (speed_mps > prev_speed_mps) {
            speed_mps = prev_speed_mps + 9.5 ;
            cout << "    accel limit speed to be:" << speed_mps <<"       " << endl;
        }
        else {
            speed_mps = prev_speed_mps - 9.5 ;
            cout << "    brake limit speed to be:" << speed_mps <<"       " << endl;
        }
    }
    if ( jerk_mpsss > 9.5 ) {
        if (accel_mpss > prev_accel_mpss) {
            accel_mpss = prev_accel_mpss + 9.5 ;
            cout << "    jerk limit accel to be:" << accel_mpss <<"       " << endl;
        }
        else {
            accel_mpss = prev_accel_mpss - 9.5 ;
            cout << "    jerk limit brake to be:" << accel_mpss <<"       " << endl;
        }
    }

    prev_accel_mpss = accel_mpss ;
    prev_speed_mps = speed_mps ;



    ref_speed = target_speed;

    if (debug_speed) {
        cout << "--------------Update-------------------" << endl;
        cout << "speed --- target :" << endl;
        cout << " " << speed
             << " --- "
             << " " << ref_speed  << endl;
    }



    ref_lane = lane;

    //clean data
    _reset_data();
}

void Vehicle::_reset_data() {
    //clean data
    //cout << "\n" << endl;
    //cout << "--------------_reset_data trajectory-------------------" << endl;
    //reset trajectory
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane;
    trajectory.target_speed = ref_speed;


    update.lane = ref_lane;
    //reset update
    update.ref_v = ref_speed;
    update.target_v = 49.50; // in mph


    if (debug_lane) {
        //cout << "vehicle: _reset_data(): trajectory.target_speed= "  << trajectory.target_speed << endl;
        
        cout << "lane_start --- lane_end --- ref_lane:" << endl;
        cout << "   " << trajectory.lane_start
             << "  ---  "
             << trajectory.lane_end
             << "  ---  "
             << ref_lane << endl;
    }


    if (debug_speed) {
        cout << "--- update.ref_v(ref_speed) --- :" << endl;
        cout << "    " << update.ref_v<< endl;
    }

    // enable collistion cost
    collider.collision = false;
    collider.distance = 10000;
    collider.changelane_gap = 10000;
    collider.target_speed = 0;
}


void Vehicle::NextState(vector<vector<double>> sensor) {

    States current_state = state;

    

    //cout << "current_state --- state:" << endl;
    //print_state(current_state);
    //cout << " --- " << endl;
    //print_state(state) ;
    //cout << "\n"  << endl;

    vector<States> states;


    //select reachable states

    // add a new element to end, len +1
    states.push_back(States::KL);
    if (debug_state) {
        cout << "--------------NextState-------------------" << endl;
        cout << "add:  KL --- new size:--- "<<states.size()<< endl;
    }
    


    if (state == States::PLCL) {
        states.push_back(States::LCL);
        states.push_back(States::PLCL);
        if (debug_state) {
            cout << "add: LCL, PLCL --- new size:--- "<<states.size()<< endl;
        }

    } 
    else if (state == States::PLCR) {
        states.push_back(States::LCR);
        states.push_back(States::PLCR);
        if (debug_state) {
            cout << "add: LCR, PLCR --- new size:--- "<<states.size()<< endl;
        }
    } 
    else {

        //cout << "ref_lane:  " << ref_lane << " --- update.lane: ---"<<update.lane<< //endl;
        //cout << "ref_speed: --- speed: " << endl;
        //cout << "  " << ref_speed
        //     << " --- "
        //     << "  " << speed << endl;

        if (ref_lane != 0) {
            //check if lane change is over before LCL again
            if (d < (2 + 4 * (ref_lane) + 2) && d > (2 + 4 * (ref_lane) - 2)
                    && speed > 20) {
                //inside lane
                states.push_back(States::PLCL);
                if (debug_state) {
                    cout << "add:  PLCL --- new size():--- "<<states.size()<< " ---speed:" << speed << " ---ref_lane:" << ref_lane<< endl;
                }
            }
        }
        if (ref_lane != 2) {
            //check if lane change is over before LCR again
            if (d < (2 + 4 * (ref_lane) + 2) && d > (2 + 4 * (ref_lane) - 2)
                    && speed > 20) {
                states.push_back(States::PLCR);
                if (debug_state) {
                    cout << "add:  PLCR --- new size():--- "<<states.size()<< " ---speed:" << speed<<" ---ref_lane:" << ref_lane<<endl;
                }
            }
        }
    }


    States min_state = States::KL;
    double  min_cost = 10000000;

    //compute cost of all reachable states
    for (int i = 0; i < states.size(); i++) {
        States n_state = states[i];

        if (debug_cost) {
            cout << " \n " << endl;
            //prepare state
            cout << "           cost for states["<<i<<"]               " << endl;
            //cout << "if states["<<i<<"] --- :"<< endl;
            //cout << "*************" <<endl; 
            print_state(n_state); 
            //cout<< "*************" <<endl;
            cout << " \n "<< endl;
        }


        //cout << " " << speed
        //     << " --- "
        //     << " " << ref_speed  << endl;

        _reset_data();
        _realise_state(n_state, sensor);

        CostFunction cost = CostFunction(this, sensor);

        double value = cost.Compute();
        if (debug_cost == true) {
            cout << " states["<<i<<"]: with cost " << value << "\n";
        }
        

        if (value < min_cost) {
            min_state = n_state;
            min_cost = value;

        }

    } // for 


    //update state
    state = min_state;
    _reset_data();
    _realise_state(state, sensor);


    //update speed
    CostFunction cost = CostFunction(this, sensor); // this  and state  ???

    //float v = cost.Compute();
    double new_value = cost.Compute();
    
    if (debug_state) {
        std::cout << "~~~~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~~~~" << endl; 
        std::cout << "~~~~~~~~~~~~ NEW STATE ~~~~~~~~~~~~" << endl;
        std::cout << "\n" << std::endl;
        print_state(state);
        std::cout << "\n" << std::endl;
        std::cout << " ~~~~~~~~~ with cost " << min_cost << " ~~~~~~~~~~" << "\n";
    }

    if (!collider.collision && 
        ref_speed < update.target_v && 
        ref_speed < 49.5) {
        // increase target speed
        update.ref_v += 0.224;
        //cout << "vehicle: NextState(): increase update.ref_v= "  << update.ref_v << endl;
    } else if (ref_speed > update.target_v && ref_speed > 0) {
         // decrease target speed
        update.ref_v -= 0.224;
        //cout << "vehicle: NextState(): minus update.ref_v= "  << update.ref_v << endl;
    }





    if (debug_speed) {
        cout << "update.ref_v - ref_speed - update.target_v:" << endl;
        cout << "    " << update.ref_v
             << " --- "
             << "  " << ref_speed
             << " --- "
             << "  " << update.target_v << endl;
    }



}

void Vehicle::_realise_state(States astate, vector<vector<double>> sensor_fusion) {


    state = astate;
    //cout << "--------------_realise_state state-------------------" << endl;
    //cout << " state:  " <<endl;
    //print_state(state);
    //cout << " \n  " <<endl;

    if (state == States::KL) {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane;
        update.lane = ref_lane;

    }

    else if (state == States::PLCL)  {
        //same lane
        trajectory.lane_start = ref_lane;

        trajectory.lane_end = ref_lane - 1;
        update.lane = ref_lane;

    }
    else if (state == States::LCL)  {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane - 1;
        update.lane = ref_lane - 1;

    }
    else if (state == States::PLCR) {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane + 1;
        update.lane = ref_lane;

    }
    else if (state == States::LCR) {
        //same lane
        trajectory.lane_start = ref_lane;
        trajectory.lane_end = ref_lane + 1;
        update.lane = ref_lane + 1;

    }
    else {
        std::cout << "STATE ERROR\n";
    }

    if (debug_lane) {
        cout << "ref_lane - trajectory.lane_start - trajectory.lane_end - update.lane:" << endl;
        cout << "  " << endl;

        cout << " -- "
             << "  " << ref_lane
             << " -- "
             << "  " << trajectory.lane_start
             << " -- "
             << "  " << trajectory.lane_end
             << " -- "
             << "  " << update.lane << endl;
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
        // *************** Safety check for speed of car at same lane *************************************
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

                if(debug_distance){
                    cout << "                                                     " << endl;
                    cout << "------------------detect collision-------------------" << endl;
                    cout << "distance (0 ~ 30) - check car speed NO.(" << i << ")" << endl;
                    cout << "  " << dist_to_collision
                         << " --- "
                         << "  " << check_speed
                         << " --- " << endl;
                } 


                //debug_distace = true;
                // if multiple cars in front and within 30m collision distance, use the closest car as archering car, and its speed as archer speed
                if (target_distance_front > dist_to_collision) {

                    // safety speed? what use ?

                    // why -2 ?
                    //target_speed_front = check_speed * MS_TO_MPH - 2;
                    target_speed_front = check_speed * MS_TO_MPH ;
                    //cout << " ------target_speed_front in mph: " << target_speed_front << endl;
                    target_distance_front = dist_to_collision;// e.g. from 10000 to 30m

                    if (debug_distance){
                        cout << " check car speed x 2.23 ( in mph)" << endl;
                        cout << "  " << target_speed_front
                             << endl;
                    }


                }
            }
        }

        // for KL don't need consider trajectory.lane_end == ref_lane
        // for PLCL, LCL, PLCR, LCR, trajectory.lane_end = trajectory.lane_start +/- 1

//#if 0
        // *************** Safety check for speed of car at next move lane *************************************
        if (checkcar_d < (2 + 4 * (trajectory.lane_end) + 2) && checkcar_d > (2 + 4 * (trajectory.lane_end) - 2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double) delta_time * check_speed);

            //check s values greater than mine and s gap
            double changelane_gap = (check_car_s - s);

            if ((trajectory.lane_end != trajectory.lane_start  && (abs(changelane_gap) < 30))
                 || ((check_car_s >= s) && (changelane_gap < 30))) {

                if (collider.distance > abs(changelane_gap)) {
                    collider.distance = abs(changelane_gap);
                    collider.collision = true;
                    collider.changelane_gap = abs(changelane_gap);
                    collider.target_speed = check_speed * MS_TO_MPH;

                    //if (abs(changelane_gap) > 30) {
                    if (abs(changelane_gap) < 30) {
                        //cout << "==============abs(changelane_gap) > 30==============================" << endl;
                        
                        //change targe speed, when need lane change but gap too small
                        if (check_car_s >= s) {
                            //car in front
                            update.target_v = check_speed * MS_TO_MPH - 2;
                            if (target_distance_lane_front > changelane_gap) {
                                target_speed_lane_front = check_speed * MS_TO_MPH;
                                target_distance_lane_front = changelane_gap;
                            }
                        } else {
                            //car in back
                            update.target_v = check_speed * MS_TO_MPH + 2;
                            if (target_distance_lane_back < changelane_gap) {
                                target_speed_lane_back = check_speed * MS_TO_MPH;
                                target_distance_lane_back = changelane_gap;
                            }
                        }
                    }
                }
            } 
            else if (!collider.collision
                       && collider.changelane_gap > changelane_gap) {
                collider.changelane_gap = changelane_gap;
                collider.target_speed = check_speed * MS_TO_MPH;
            }
        }// if sensor a car on other lane
//#endif

    }// end for

//#if 0
    //Safety Speed check
    if (state == States::PLCL || state == States::PLCR) {
        //safety speed adjust
        if (target_speed_lane_back != 0 && update.target_v < target_speed_lane_back) {
            update.target_v = target_speed_lane_back;
        }
        if (target_speed_lane_front != 0 && update.target_v > target_speed_lane_front) {
            update.target_v = target_speed_lane_front;
        }
    }

//#endif

    if (target_speed_front != 0 && update.target_v > target_speed_front) {
        //update.target_v = target_speed_front - 2;
        update.target_v = target_speed_front - 1;
        //cout << "  target speed : -1 mph from front: " << endl ;

        //cout << "-----------------------------------------------------" << endl;
        //cout << "                                                     " << endl;
    }
    else {
        //cout << "  no car front to limit target speed ! " << endl ;
    }

    if (debug_speed) {
        cout << "  " << update.target_v
             << endl;
    }

}







