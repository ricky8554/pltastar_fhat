#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
#include "LSS_LRTA.h"
// #include "PLTASTAR.h"
// #include "LSS_LRTA_FHAT.h"
// #include "PLTASTAR_FHAT.h"
#include "state.h"

using namespace std;

#define rate 30
#define bord 100



class Obstacle
{
  private:
    vector<DynamicObstacle> dynamicObstacles;
    unordered_set<StaticObstacle> staticObstacles;
    vector<State> path;
    State start, goal;
    Plan* planner = 0;
    int termination;
    int bordw;
    int bordh;
    int updaterate;
    int plan;
    int index;
    int mode;//0 is lsslrtastar 1 is plrtastar
    void MoveObstacle();
    
    unordered_set<State> map;

  public:
    // Default constructor
    Obstacle()
        : termination(1), updaterate(rate), plan(1),index(0)
    {
        srand(time(NULL));
    };

    // Explicit constructor
    Obstacle(vector<DynamicObstacle> dynamicObstacles, unordered_set<StaticObstacle> staticObstacles)
        : dynamicObstacles(dynamicObstacles), staticObstacles(staticObstacles), termination(1), updaterate(rate), plan(1),index(0)
    {
        srand(time(NULL));
    };

    int MoveObstacle(int node,int LookAhead);

    void addDynamicObstacle(DynamicObstacle);

    void addStaticObstacle(double x1, double y1);


    void setStartPoint(double x, double y);

    void setGoalPoint(double x, double y);

    void setBoard(double w, double h)
    {
        bordw = w; 
        bordh = h;
    };


    void scheduledChanged();

    vector<State> getPath()
    {
        return path;
    };

    void terminate();

    State getStatePoint();

    int collisionChecking(int index, double radius, int x, int y);

    vector<Dynamicxy> getDynamicObstacle();

    unordered_set<StaticObstacle> getStaticObstacle();

    
    unordered_set<State> getSTATE()
    {
        mtx1.lock();
        unordered_set<State> a = map;
        mtx1.unlock();
        return a;
    }

    vector<DynamicObstacle> getDynamicObstacle1()
    {
        return dynamicObstacles;
    };

    void setDynamicObstacle(vector<DynamicObstacle> a)
    {
        dynamicObstacles = a;
    };

    void initialize();

    void initialize1(int a);

    void update();

    void update1();

    void setMode(int MODE)
    {
        mode = MODE;
    }


    mutex mtx, mtx1;


    vector<thread> threads;
};

#endif