#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;


class CostFunction {
  public:
    const double COLLISION  = pow(10,6); //pow(10, 2); //pow(10,6);
    const double BUFFER     = pow(10,5); //pow(10, 2); //pow(10,5);
    //const double REACH_GOAL = pow(10,5);
    const double CHANGELANE    = pow(10, 4); //pow(10, 2); //pow(10, 4);
    const double INEFFICIENCY = pow(10, 2); //2
    const double TARGET = pow(10, 2);


    const double DESIRED_BUFFER = 1;//1.5;
    const double PLANNING_HORIZON = 2;

    vector<vector<double>> sensor_fusion;
    Vehicle *vehicle;

    /**
     * Constructor
     */
    CostFunction(Vehicle *v, vector<vector<double>> s);

    /*
     * Cost functions
     */
    double ChangeLane();
    double DistanceGoalLane();
    double Inefficiency();
    double Collision();
    double Buffer();
    double Target();

    double Compute();


};

#endif