#ifndef __SKL_H__
#define __SKL_H__

#include <stdio.h>
#include <iostream>
#include "PQueue.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <float.h>
#include "object.h"

using namespace std;

class Skeloton
{
  private:
    int boardw, boardh, startTime;
    double error;
    State start, goal;
    PQueue<State> open;
    PQueue<State> goalQ;
    unordered_set<State> opencheck;
    unordered_set<State> close;
    unordered_set<State> expandState;
    unordered_map<State, State> parent;
    vector<DynamicObstacle> dynamicObstacles;
    vector<StaticObstacle> staticObstacles;
    vector<State> path;
    float *htable;

  public:
    // Default constructor
    Skeloton()
    {
        error = 0;
        ios_base::sync_with_stdio(false);
    };

    // Explicit constructor
    Skeloton(State start, State goal)
        : start(start), goal(goal)
    {
        error = 0;
        ios_base::sync_with_stdio(false);
    };

    void setStartGoal(State starta, State goala, int bordw, int bordh);

    double cost(State &s, State &s1);

    int checkValid(State &s, int timestamp);

    double h_value(State &s);

    int getDistance(State &s);

    int ASTAR();

    int Dijkstra();

    int LSS_LRTASTAR(State start);

    void setStatic(vector<StaticObstacle> &s);

    void setDynamic(vector<DynamicObstacle> &d);

    State pickBest();

    void constructHtable();
    bool ccheck(int x, int y);
    int getindex(int x, int y);
    
    int cost_obs(State &s, int timestamp);

    vector<State> getPath();

    unordered_set<State> getSTATE()
    {
        return expandState;
    }

    mutex mtx;
};

#endif