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

void Obstacle::addDynamicObstacle(DynamicObstacle &d)
{
    dynamicObstacles.push_back(d);
}

void Obstacle::update_dynamic(DynamicObstacle &dynamic)
{

    double speed, heading;
    dynamic.estimate_h = heading = dynamic.heading;
    dynamic.estimate_s = speed = dynamic.speed;
    dynamic.x += cos(heading) * speed;
    dynamic.y += sin(heading) * speed;
    if (--dynamic.remain == 0)
    {
        dynamic_path_change = 1;
        dynamic.current += 1;
        movement m = dynamic.instructions[dynamic.current % dynamic.total];
        dynamic.heading = m.heading;
        dynamic.speed = m.speed;
        dynamic.remain = m.steps;
    }
}

void Obstacle::update(bool graph_mode)
{
    dynamic_path_change = 0;
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

int Obstacle::replan()
{
    State s = path[index], ps = start;
    int factor1;
    double timeinterval = factor1 = 10;
    int cx = ps.x, cy = ps.y, fx = s.x, fy = s.y, dt = 1;
    double diffx = (fx - cx) / timeinterval, diffy = (fy - cy) / timeinterval;

    double p_collide = 0;
    for (int i = 1; i <= factor1; i++)
    {
        double x = cx + diffx * i, y = cy + diffy * i;
        double temp_p_collide = 0, p_not_collide = 1;
        for (DynamicObstacle dynamic : dynamicObstacles)
        {

            double dhead = dynamic.estimate_h, dspeed = dynamic.estimate_s;
            int dyt = (dt > 20) ? 20 : dt;
            double dtime = (dt - 1 + i / timeinterval);
            double dyx = cos(dhead) * dspeed * dtime + dynamic.x, dyy = sin(dhead) * dspeed * dtime + dynamic.y;
            double powt = pow(1.2, dyt); //change 1.2 to any
            double range = 0.5 * powt;
            double dfx = dyx - x, dfy = dyy - y;
            if (dfx * dfx + dfy * dfy < range * range)
            {
                p_not_collide *= (1 - 1 / powt);
            }
        }
        temp_p_collide = 1 - p_not_collide;
        if (temp_p_collide > p_collide)
        {
            p_collide = temp_p_collide;
        }
    }

    return p_collide * 200 > s.tempc * threshold_factor;
}

void Obstacle::MoveObstacle()
{
    if (plan)
    {
        initialize(0);
    }

    while (termination)
    {
        mtx.lock();
        for (DynamicObstacle &dynamic : dynamicObstacles)
            update_dynamic(dynamic);

        if (path.size() > index)
            start = path[index++];

        mtx.unlock();

        unordered_set<State> s = getSTATE();
        if (path.size() <= index)
        {

            clock_t begin = clock();
            update(1);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            cout << elapsed_secs << " SECS" << endl;
        }
        else if (mode_dynamic && replan())
        {
            int erase = path.size() - index;
            path.erase(path.end() - erase, path.end());
            planner->setLookAhead(node_per_step);
            clock_t begin = clock();
            update(1);
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
            cost += 200;

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

vector<DynamicObstacle> Obstacle::getDynamicObstacle()
{
    vector<DynamicObstacle> temp;
    //mtx.lock();
    for (DynamicObstacle &dynamic : dynamicObstacles)
        temp.push_back(dynamic);
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
