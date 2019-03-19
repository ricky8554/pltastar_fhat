#include "obstacle.h"
#include "iostream"
#include <chrono>
#include "math.h"
#include <iomanip>

using namespace std;

#define PIR 3.141592653589793
#define RESET "\033[0m"
#define RED "\033[31m"
#define sqrt2 1.41421356237

void Obstacle::addStaticObstacle(double x1, double y1)
{
    staticObstacles.insert(StaticObstacle(x1, y1));
}

void Obstacle::addDynamicObstacle(double maxspeed, double minspeed, double radius,int obs, double x, double y)
{
    obs = obs -48;
    dynamicObstacles.push_back(DynamicObstacle(maxspeed, minspeed, radius, obs , x, y));
}

void Obstacle::update_dynamic(DynamicObstacle &d)
{

    int x,y;
    for (int i = 0; i < 2; i++)
    {
        
        int addx = d.angle/10 - 1;
        int addy = (d.angle%10) - 1;
        x = d.x + addx;
        y = d.y + addy;
        dummy_static_obs.set(x, y);
        if (staticObstacles.find(dummy_static_obs) == staticObstacles.end() && x >= 0 && x < bordw && y >= 0 && y < bordh)
        {
            d.x = x;
            d.y = y;
            return;
        }
        else
        {
            d.angle = (addx == 0) ? 1 : (addx == 1) ? 0 : 2;
            d.angle =d.angle * 10 +  ((addy == 0) ? 1 : (addy == 1) ? 0 : 2);
        }
    }
}

void Obstacle::update(bool graph_mode)
{
    
    planner->set_time(start.time);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    if (mode_dynamic)
    {
        dynamic_lookahead = a.size();
        planner->setLookAhead(dynamic_lookahead * node_per_step);
        path.insert(path.end(), a.rbegin(), a.rend());
    }
    else
        path.push_back(a.back());

    if (graph_mode)
    {
        mtx1.lock();
        map = planner->getSTATE();
        mtx1.unlock();
    }
}



void Obstacle::initialize(int LOOKAHEAD)
{
    
    if (mode == 0 || mode == 6)
        planner = new Lss_Lrta();
    else if (mode == 1 || mode == 7)
        planner = new PLTASTAR();
    else if (mode == 2 || mode == 8)
        planner = new Lss_Lrta_Fhat();
    else if (mode == 3 || mode == 9)
        planner = new PLTASTAR_FHAT();
    else if (mode == 4 || mode == 10)
        planner = new PLTASTAR_MOD();
    else if (mode == 5 || mode == 11)
        planner = new PLTASTAR_FHAT_MOD();
    
    if (mode > 5)
        mode_dynamic = true;

    if (LOOKAHEAD)
        planner->setLookAhead(LOOKAHEAD);

    planner->setStatic(staticObstacles);
    planner->setStartGoal(start, goal, bordw, bordh);

    update(!LOOKAHEAD);


}

void Obstacle::MoveObstacle()
{
    int step = 1;
    if (plan)
    {
        initialize(0);
    }
    
    while (termination)
    {
        
        bool collide = false;
        mtx.lock();
        if (path.size() > index)
            start = path[index++];

        for (DynamicObstacle &dynamic : dynamicObstacles)
        {
            update_dynamic(dynamic);
            if (dynamic.x == start.x && dynamic.y == start.y)
                collide = true;
        }

        mtx.unlock();

        if (collide)
            start = State(0,bordh-1,start.time);

        unordered_set<State> s = getSTATE();
        if (path.size() <= index)
        {
            clock_t begin = clock();
            update(1);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            cout << elapsed_secs << " SECS Step " << step << endl;
        }
        step += 1;
        

        this_thread::sleep_for(std::chrono::milliseconds(updaterate));
    }
}

int Obstacle::MoveObstacle(int Maxstep, int LookAhead)
{
    int cost = 0;
    int step = 1;
    node_per_step = LookAhead;
    initialize(LookAhead);

    while (Maxstep > step++)
    {
        bool collide = false;
        if (path.size() > index)
        {
            start = path[index++];
        }

        for (DynamicObstacle &dynamic : dynamicObstacles)
        {
            update_dynamic(dynamic);
            if (dynamic.x == start.x && dynamic.y == start.y)
                collide = true;
        }

        if (collide)
        {
            start = State(0,bordh-1,start.time);
            cost += 200;
        }

        if (start.x != goal.x || start.y != goal.y)
            cost += 1;

        if (path.size() <= index)
            update(0);
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
