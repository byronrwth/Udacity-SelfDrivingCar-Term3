#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "hybrid_breadth_first.h"

using namespace std;


/**
 * Initializes HBF
 */
HBF::HBF() {

}

HBF::~HBF() {}

/********no optimizaion:

found path to goal in 12606 expansions

**********/

/*** optimizaion 1: ***********

bool HBF::compare_maze_s(const HBF::maze_s & lhs, const HBF::maze_s & rhs) {
    return lhs.f < rhs.f;
}

double HBF::heuristic(double x, double y, vector<int> goal){
  return fabs(y - goal[0]) + fabs(x - goal[1]); //return grid distance to goal
}

====> found path to goal in 10082 expansions

****/

/*******optimizaion 2: **********

To get an even lower number of expansions, try reducing NUM_THETA_CELLS in hybrid_breadth_first.h to reduce the total number of cells that can be searched in the closed array. Be careful though! Making NUM_THETA_CELLS too small might result in the algorithm being unable to find a path through the maze.


************************/

/*******optimizaion 3: **********

Another possibility for improvement is to use the regular A* algorithm to assign a cost value to each grid cell. This grid of costs can then be used as the heuristic function, which will lead to an extremely efficient search.

***************/


int HBF::theta_to_stack_number(double theta){
  /*
  Takes an angle (in radians) and returns which "stack" in the 3D configuration space
  this angle corresponds to. Angles near 0 go in the lower stacks while angles near 
  2 * pi go in the higher stacks.
  */

  double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) % NUM_THETA_CELLS;
  return stack_number;
}


int HBF::idx(double float_num) {
  /*
  Returns the index into the grid for continuous position. So if x is 3.621, then this
  would return 3 to indicate that 3.621 corresponds to array index 3.
  */

  return int(floor(float_num));
}


vector<HBF::maze_s> HBF::expand(HBF::maze_s state) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;

  int g2 = g+1;
  vector<HBF::maze_s> next_states;
  for(double delta_i = -35; delta_i < 40; delta_i+=5)
  {

    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if(theta2 < 0)
    {
        theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta);
    double y2 = y + SPEED * sin(theta);
    HBF::maze_s state2;
    //state2.f = g2 + heuristic(x2, y2, goal);
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);

  }
  return next_states;
}

vector< HBF::maze_s> HBF::reconstruct_path(vector< vector< vector<HBF::maze_s> > > came_from, vector<double> start, HBF::maze_s final){

    vector<maze_s> path = {final};
    
    int stack = theta_to_stack_number(final.theta);

    maze_s current = came_from[stack][idx(final.x)][idx(final.y)];
    
    stack = theta_to_stack_number(current.theta);
    
    double x = current.x;
    double y = current.y;
    while( x != start[0] || y != start[1] )
    {
        path.push_back(current);
        current = came_from[stack][idx(x)][idx(y)];
        x = current.x;
        y = current.y;
        stack = theta_to_stack_number(current.theta);
    }
    
    return path;

}

HBF::maze_path HBF::search(vector< vector<int> > grid, vector<double> start, vector<int> goal) {
  /*
  Working Implementation of breadth first search. Does NOT use a heuristic
  and as a result this is pretty inefficient. Try modifying this algorithm 
  into hybrid A* by adding heuristics appropriately.
  */

  vector< vector< vector<int> > > closed(NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  vector< vector< vector<maze_s> > > came_from(NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())));
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  int g = 0;

  maze_s state;
  state.g = g;
  state.x = start[0];
  state.y = start[1];
  //state.f = g + heuristic(state.x, state.y, goal);
  state.theta = theta;

  closed[stack][idx(state.x)][idx(state.y)] = 1;
  came_from[stack][idx(state.x)][idx(state.y)] = state;
  int total_closed = 1;
  vector<maze_s> opened = {state};
  bool finished = false;
  while(!opened.empty())
  {

    //sort(opened.begin(), opened.end(), compare_maze_s);
    maze_s current = opened[0]; //grab first elment
    opened.erase(opened.begin()); //pop first element

    int x = current.x;
    int y = current.y;

    if(idx(x) == goal[0] && idx(y) == goal[1])
    {
      cout << "found path to goal in " << total_closed << " expansions" << endl;
      maze_path path;
      path.closed = closed;
      path.came_from = came_from;
      path.final = current;
      return path;

    }
    vector<maze_s> next_state = expand(current);

    for(int i = 0; i < next_state.size(); i++)
    {
      int g2 = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))
      {
        //invalid cell
        continue;
      }
      int stack2 = theta_to_stack_number(theta2);

      if(closed[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0)
      {

        opened.push_back(next_state[i]);
        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        total_closed += 1;
      }


    }

  }
  cout << "no valid path." << endl;
  HBF::maze_path path;
  path.closed = closed;
  path.came_from = came_from;
  path.final = state;
  return path;

}

