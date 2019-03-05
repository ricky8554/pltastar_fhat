#include "obstacle.h"
#include "iostream"
#include <chrono>
#include "math.h"
#include <iomanip>

using namespace std;

#define PIR 3.14159265
#define RESET "\033[0m"
#define RED "\033[31m"
#define sqrt2 1.41421356237

void Obstacle::addStaticObstacle(double x1, double y1)
{
    staticObstacles.insert(StaticObstacle(x1, y1));
}

void Obstacle::addDynamicObstacle(double maxspeed, double minspeed, double radius, double x, double y)
{
    dynamicObstacles.push_back(DynamicObstacle(maxspeed, minspeed, radius, x, y));
}

void Obstacle::update_dynamic(DynamicObstacle &d)
{
    int x;
    for(int i = 0; i < 2; i ++)
    {
        x = d.x + cos(d.angle);
        dummy_static_obs.set(x,d.y);
        if(staticObstacles.find(dummy_static_obs) == staticObstacles.end() && x >= 0 && x < bordw )
        {
            d.x = x;
            return;
        }
        else
            d.angle = (d.angle) ? 0: PIR;
    }
}


void Obstacle::initialize()
{
    if (mode == 0 || mode == 4)
    {
        planner = new Lss_Lrta();
    }
    else if (mode == 1 || mode == 5)
    {
        planner = new PLTASTAR();
    }
    else if (mode == 2 || mode == 6)
    {
        planner = new Lss_Lrta_Fhat();
    }
    else if (mode == 3 || mode == 7)
    {
        planner = new PLTASTAR_FHAT();
    }
    if(mode > 3)
        mode_dynamic = true;
    planner->setStatic(staticObstacles);
    planner->setStartGoal(start, goal, bordw, bordh);
    planner->set_time(start.time);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    if(mode_dynamic)
    {
        dynamic_lookahead = a.size();
        planner->setLookAhead(dynamic_lookahead * node_per_step);
        path.insert(path.end(), a.rbegin(), a.rend());
    }
    else 
        path.push_back(a.back());

    mtx1.lock();
    map = planner->getSTATE();
    mtx1.unlock();
}
void Obstacle::update()
{
    planner->set_time(start.time);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    if(mode_dynamic)
    {
        dynamic_lookahead = a.size();
        planner->setLookAhead(dynamic_lookahead * node_per_step);
        path.insert(path.end(), a.rbegin(), a.rend());
    }
    else 
        path.push_back(a.back());

    mtx1.lock();
    map = planner->getSTATE();
    mtx1.unlock();
}

void Obstacle::update1()
{
    planner->set_time(start.time);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    if(mode_dynamic)
    {
        dynamic_lookahead = a.size();
        planner->setLookAhead(dynamic_lookahead * node_per_step);
       
        path.insert(path.end(), a.rbegin(), a.rend());
        
    }
    else 
        path.push_back(a.back());
}



void Obstacle::initialize1(int LookAhead)
{
    if(planner)
        delete planner;
    if (mode == 0 || mode == 4)
    {
        planner = new Lss_Lrta();
    }
    else if (mode == 1 || mode == 5)
    {
        planner = new PLTASTAR();
    }
    else if (mode == 2 || mode == 6)
    {
        planner = new Lss_Lrta_Fhat();
    }
    else if (mode == 3 || mode == 7)
    {
        planner = new PLTASTAR_FHAT();
    }

    if(mode > 3)
        mode_dynamic = true;
    
    planner->setLookAhead(LookAhead);
    planner->setStatic(staticObstacles);
    planner->setStartGoal(start, goal, bordw, bordh);
    planner->set_time(start.time);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    if(mode_dynamic)
    {
        dynamic_lookahead = a.size();
        planner->setLookAhead(dynamic_lookahead * node_per_step);
        path.insert(path.end(), a.rbegin(), a.rend());
    }
    else 
        path.push_back(a.back());
}

void Obstacle::MoveObstacle()
{
    if (plan)
    {
        initialize();
    }

    while (termination)
    {
        mtx.lock();
        for (DynamicObstacle & dynamic:dynamicObstacles)
            update_dynamic(dynamic);

        if (path.size() > index)
            start = path[index++];
        mtx.unlock();

        unordered_set<State> s = getSTATE();
        if (path.size() <= index)
        {

            clock_t begin = clock();
            update();
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            cout << elapsed_secs << " SECS" << endl;
        }
        this_thread::sleep_for(std::chrono::milliseconds(updaterate));
    }
}

int Obstacle::MoveObstacle(int Maxstep, int LookAhead)
{
    int cost = 0;
    int step = 1;

    initialize1(LookAhead);

    while (Maxstep > step++)
    {
        bool collide = false;
        if (path.size() > index)
        {
            start = path[index++];
        }

        for (DynamicObstacle & dynamic:dynamicObstacles)
        {
            update_dynamic(dynamic);
            if (dynamic.x == start.x && dynamic.y == start.y)
                collide = true;
        }

        if (collide)
            cost += 200;

        if (start.x != goal.x || start.y != goal.y)
            cost += 1;

        if (path.size() <= index)
            update1();
    }
    if (path.size() > index)
    {
        start = path[index++];
    }
    if (start.x != goal.x || start.y != goal.y)
    {
        cost += 1;
    }
    return cost;
}

void Obstacle::scheduledChanged()
{
    threads.push_back(thread([=] { MoveObstacle(); }));
}

void Obstacle::terminate()
{
    termination = 0;
    if (threads.size())
        threads[0].join();
}

vector<Dynamicxy> Obstacle::getDynamicObstacle()
{
    vector<Dynamicxy> temp;
    //mtx.lock();
    for (DynamicObstacle dynamic : dynamicObstacles)
        temp.push_back(dynamic.getxy());
    //mtx.unlock();
    return temp;
}

unordered_set<StaticObstacle> Obstacle::getStaticObstacle()
{
    return staticObstacles;
}

State Obstacle::getStatePoint()
{
    return start;
}

void Obstacle::setStartPoint(double x, double y)
{
    start = State(x, y, 0);
}

void Obstacle::setGoalPoint(double x, double y)
{
    goal = State(x, y, 0);
}
