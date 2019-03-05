#include "obstacle.h"
#include "iostream"
#include <chrono>
#include "math.h"
#include <iomanip>

using namespace std;

#define PIR 3.14159265 / 180
#define RESET "\033[0m"
#define RED "\033[31m"
#define sqrt2 1.41421356237

void Obstacle::addStaticObstacle(double x1, double y1)
{
    staticObstacles.insert(StaticObstacle(x1, y1));
}

void Obstacle::addDynamicObstacle(DynamicObstacle d)
{
    dynamicObstacles.push_back(d);
}

int Obstacle::collisionChecking(int index, double radius, int x, int y)
{
    // for (StaticObstacle staticobs : staticObstacles)
    // {
    //     if (staticobs.x == x && staticobs.y == y)
    //     {
    //         if (!dynamicObstacles[index].angle)
    //             dynamicObstacles[index].angle = PIR * 180;
    //         else
    //             dynamicObstacles[index].angle = 0;

    //         return 1;
    //     }
    // }
    // if (x < 0 || x >= bordw || y < 0 || y > bordh)
    // {
    //     if (!dynamicObstacles[index].angle)
    //         dynamicObstacles[index].angle = PIR * 180;
    //     else
    //         dynamicObstacles[index].angle = 0;
    //     return 1;
    // }

    return 0;
}

// static double drand(double Max, double Min)
// {
//     return ((((double)rand()) / RAND_MAX) * (Max - Min) + Min);
// }

// static double h_value(State &s, State &goal)
// {
//     int x = abs(s.x - goal.x), y = abs(s.y - goal.y), min, dif;
//     if (x > y)
//     {
//         min = y;
//         dif = x;
//     }
//     else
//     {
//         min = x;
//         dif = y;
//     }
//     return sqrt2 * min + dif;
// }

void Obstacle::initialize()
{
    if (mode == 0)
    {
        planner = new Lss_Lrta();
    }
    // else if (mode == 1)
    // {
    //     planner = new PLTASTAR();
    // }
    // else if (mode == 2)
    // {
    //     planner = new Lss_Lrta_Fhat();
    // }
    // else if (mode == 3)
    // {
    //     planner = new PLTASTAR_FHAT();
    // }
    planner->setStatic(staticObstacles);
    planner->setStartGoal(start, goal, bordw, bordh);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    path.push_back(a[a.size() - 1]);
    mtx1.lock();
    map = planner->getSTATE();
    mtx1.unlock();
}
void Obstacle::update()
{

    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(path[index - 1]);
    vector<State> a = planner->getPath();
    path.push_back(a[a.size() - 1]);

    mtx1.lock();
    map = planner->getSTATE();
    mtx1.unlock();
}

void Obstacle::update1()
{
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(path[index - 1]);
    vector<State> a = planner->getPath();
    path.push_back(a.back());
}

void Obstacle::initialize1(int LookAhead)
{
    if(planner)
        delete planner;
    if (mode == 0)
    {
        planner = new Lss_Lrta();
    }
    // else if (mode == 1)
    // {
    //     planner = new PLTASTAR();
    // }
    // else if (mode == 2)
    // {
    //     planner = new Lss_Lrta_Fhat();
    // }
    // else if (mode == 3)
    // {
    //     planner = new PLTASTAR_FHAT();
    // }
    planner->setLookAhead(LookAhead);
    planner->setStatic(staticObstacles);
    planner->setStartGoal(start, goal, bordw, bordh);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    path.push_back(a[a.size() - 1]);
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
        double speed, heading;
        for (DynamicObstacle dynamic: dynamicObstacles )
        {
            dynamic.estimate_h = heading = dynamic.heading;
            dynamic.estimate_s = speed = dynamic.speed;
            dynamic.x += cos(heading) * speed;
            dynamic.y += sin(heading) * speed;
            if(--dynamic.remain == 0)
            {
                dynamic.current += 1;
                movement m = dynamic.instructions[dynamic.current % dynamic.total];
                dynamic.heading = m.heading;
                dynamic.speed = m.speed;
                dynamic.remain = m.steps;
            }
        }

        if (path.size() > index)
        {
            start = path[index++];
        }
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
        double speed, heading, collide = false;
        if (path.size() > index)
        {
            start = path[index++];
        }

        for (DynamicObstacle dynamic: dynamicObstacles )
        {
            dynamic.estimate_h = heading = dynamic.heading;
            dynamic.estimate_s = speed = dynamic.speed;
            dynamic.x += cos(heading) * speed;
            dynamic.y += sin(heading) * speed;
            if (dynamic.x == start.x && dynamic.y == start.y)//fix
            {
                collide = true;
            }
        }
        if (collide)
        {
            cost += 200;
        }

        if (start.x != goal.x || start.y != goal.y)
        {
            cost += 1;
        }

        if (path.size() <= index)
        {

            //clock_t begin = clock();
            update1();
            //clock_t end = clock();
            //double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            //cout << elapsed_secs << " SECS" << endl;
        }
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
    start = State(x, y,0,0, 0);
}

void Obstacle::setGoalPoint(double x, double y)
{
    goal = State(x, y,0,0, 0);
}
