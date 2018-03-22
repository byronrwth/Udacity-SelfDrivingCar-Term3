#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;


class CostFunction {
  public:
    const double COLLISION             = pow(10,6); //pow(10, 2); //pow(10,6);
    const double CHANGELANE_DANGER     = pow(10,5); //pow(10, 2); //pow(10,5);
    const double CHANGELANE_COMFORT    = pow(10, 4); //pow(10, 2); //pow(10, 4);
    const double INEFFICIENCY = pow(10, 2); //2
    const double TARGET = 100; //pow(10, 2);


    const double CHANGELANE_DURATION = 1;//1.5;
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
    double changelane_comfort();
    //double DistanceGoalLane();
    double Inefficiency();
    double Collision();
    double changelane_danger();
    double Target();

    double Compute();


};

#endif