#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
// #include "base_a.h"
#include "LSS_LRTA.h"
#include "LSS_LRTA_FHAT.h"
#include "PLTASTAR.h"
#include "PLTASTAR_FHAT.h"
#include "PLTASTAR_MOD.h"
#include "PLTASTAR_FHAT_MOD.h"
#include "state.h"

using namespace std;
#define bord 100



class Obstacle
{
  private:
    vector<DynamicObstacle> dynamicObstacles;
    unordered_set<StaticObstacle> staticObstacles;
    StaticObstacle dummy_static_obs;
    vector<State> path;
    State start, goal;
    Plan* planner = 0;
    double threshold_factor = 1.1;
    int dynamic_path_change = 0;
    int max_check = 100;
    int termination;
    int bordw;
    int bordh;
    int updaterate = 300;
    int plan;
    int index;
    int mode;//0 is lsslrtastar 1 is plrtastar
    bool mode_dynamic = false;
    int dynamic_lookahead = 1;
    int node_per_step = 1000;
    void MoveObstacle();
    
    unordered_set<State> map;

  public:
    // Default constructor
    Obstacle()
        : termination(1), plan(1),index(0)
    {
        srand(time(NULL));
    };

    // Explicit constructor
    Obstacle(vector<DynamicObstacle> dynamicObstacles, unordered_set<StaticObstacle> staticObstacles)
        : dynamicObstacles(dynamicObstacles), staticObstacles(staticObstacles), termination(1), plan(1),index(0)
    {
        srand(time(NULL));
    };

    int MoveObstacle(int node,int LookAhead);
    
    double get_probability(DynamicObstacle &dynamic, double x, double y, double t);
    
    void addDynamicObstacle(DynamicObstacle &d);

    void addStaticObstacle(double x1, double y1);


    void setStartPoint(double x, double y);

    void setGoalPoint(double x, double y);

    void setBoard(double w, double h)
    {
        bordw = w; 
        bordh = h;
    };

    void setRate(int r)
    {
        updaterate = r;
    };

    void setLookAhead(int l)
    {
        node_per_step = l;
    };


    void scheduledChanged();

    vector<State> getPath()
    {
        return path;
    };

    void terminate();

    State getStatePoint();

    int collisionChecking(int index, double radius, int x, int y);

    vector<DynamicObstacle> getDynamicObstacle();

    unordered_set<StaticObstacle> getStaticObstacle();

    
    unordered_set<State> getSTATE()
    {
        mtx1.lock();
        unordered_set<State> a = map;
        mtx1.unlock();
        return a;
    }

    int get_cost(State &s, State ps);

    vector<DynamicObstacle> getDynamicObstacle1()
    {
        return dynamicObstacles;
    };

    void setDynamicObstacle(vector<DynamicObstacle> a)
    {
        dynamicObstacles = a;
    };

    void initialize(int LOOKAHEAD);

    int replan();


    void update(bool graph_mode);

    void update_dynamic(DynamicObstacle &d);

    void setMode(int MODE)
    {
        mode = MODE;
    }


    mutex mtx, mtx1;


    vector<thread> threads;
};

#endif