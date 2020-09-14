#include "executive.h"
#include "iostream"
#include <chrono>
#include "math.h"
#include <iomanip>

using namespace std;

#define PIR 3.141592653589793
#define RESET "\033[0m"
#define RED "\033[31m"
#define sqrt2 1.41421356237

/**
 * Add Static Obstacle info to executive
 * 
 * @param x1 x-axis of a static obstacle.
 * @param y1 y-axis of a static obstacle.
 */
void Executive::addStaticObstacle(double x1, double y1)
{
    staticObstacles.insert(StaticObstacle(x1, y1));
}

/**
 * Add Dynamic Obstacle info to executive
 * 
 * @param d information(x,y,heading,speed) of a dynamic obstacle.
 */
void Executive::addDynamicObstacle(DynamicObstacle &d)
{
    dynamicObstacles.push_back(d);
}

/**
 * update a dynamic obstacles to next state according to its heading and speed.
 * 
 * @param dynamic information(x,y,heading,speed) of a dynamic obstacle.
 */
void Executive::update_dynamic(DynamicObstacle &dynamic)
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

/**
 * request a new plan/path from the planner base on current/future(dynamic) state.
 * 
 * @param graph_mode True If simulator is on Else False
 */
void Executive::update(bool graph_mode)
{
    dynamic_path_change = 0;
    planner->set_time(start.time);
    planner->setDynamic(dynamicObstacles);
    plan = planner->plan(start);
    vector<State> a = planner->getPath();
    if (mode_dynamic && mode < 12)
    {
        int size = a.size();
        dynamic_lookahead = (dynamic_lookahead + 1 > size) ? size : dynamic_lookahead + 1;
        planner->setLookAhead(dynamic_lookahead * node_per_step);
        for (int i = 0; i < dynamic_lookahead; i++)
            path.push_back(a[size - 1 - i]);
    }
    else if(mode_dynamic)
    {
        int size = a.size();
        planner->setLookAhead(node_per_step);
        for (int i = 0; i < size; i++)
            path.push_back(a[size - 1 - i]);
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

/**
 * Initialize the executive based on the given information from simulator or controller.
 * 
 * @param LOOKAHEAD number of iteration/lookaheas allows planner to have.
 */
void Executive::initialize(int LOOKAHEAD)
{
    dynamic_lookahead = 1;
    // cerr << "mode "<<mode << endl;
    if (planner)
        delete planner;
    if (mode == -1)
        planner = new Base_plan();
    else if (mode == 0 || mode == 6)
        planner = new Lss_Lrta();
    else if (mode == 1 || mode == 7 || mode == 12)
        planner = new PLTASTAR();
    else if (mode == 2 || mode == 8)
        planner = new Lss_Lrta_Fhat();
    else if (mode == 3 || mode == 9 || mode == 13)
        planner = new PLTASTAR_FHAT();
    else if (mode == 4 || mode == 10)
        planner = new PLTASTAR_MOD();
    else if (mode == 5 || mode == 11)
        planner = new PLTASTAR_FHAT_MOD();

    planner->setLookAhead(node_per_step);

    if (mode > 5 || mode == -1)
        mode_dynamic = true;

    if (LOOKAHEAD)
        planner->setLookAhead(LOOKAHEAD);

    planner->setStatic(staticObstacles);
    planner->setStartGoal(start, goal, bordw, bordh);
    update(!LOOKAHEAD);
}

/**
 * Helping function for calculating the probability of a dynamic obstacles crash with agent.
 * 
 * @param dynamic information(x,y,heading,speed) of a dynamic obstacle.
 * @param x agent x-axis at t time
 * @param y agent y-axis at t time
 * @param t given time for agent to be at x, y
 * @return Probability of a dynamic obstacle crash with the agent.
 */
double Executive::get_probability(DynamicObstacle &dynamic, double x, double y, double t)
{
    double dhead = dynamic.estimate_h, dspeed = dynamic.estimate_s;
    double mx = cos(dhead) * dspeed * t + dynamic.x, my = sin(dhead) * dspeed * t + dynamic.y;
    double stderr = 1 + 0.1 * t;
    double variance = stderr * stderr;
    double dx = x - mx, dy = y - my;
    // double pxy = ((x - mx) * (y - my)) / variance;
    double first = 1 / (2.0 * M_PI * variance);
    double second = -1 / 2.0;
    double third = ((dx * dx + dy * dy)) / variance;
    return first * exp(second * third);
}

/**
 * Helping function for calculating the cost of the agent travel from one state to another.
 * 
 * @param s current state info.
 * @param ps previous state info
 * @return Cost between two states.
 */
int Executive::get_cost(State &s, State ps)
{
    int factor1;
    double timeinterval = factor1 = 10;
    int cx = ps.x, cy = ps.y, fx = s.x, fy = s.y, dt = s.time - start.time;
    double diffx = (fx - cx) / timeinterval, diffy = (fy - cy) / timeinterval;
    double p_collide = 0;
    for (int i = 1; i <= factor1; i++)
    {
        double x = cx + diffx * i, y = cy + diffy * i;
        double temp_p_collide = 0, p_not_collide = 1;

        for (DynamicObstacle &dynamic : dynamicObstacles)
            p_not_collide *= (1 - get_probability(dynamic, x, y, (dt - 1 + i / timeinterval)));

        temp_p_collide = 1 - p_not_collide;

        if (temp_p_collide > p_collide)
            p_collide = temp_p_collide;
    }
    if (p_collide > 1)
        p_collide = 1;
    return p_collide * 200;
}


/**
 * Helping function for deciding weather a plan need to be replaned.
 * Replan only happen in dynamic mode as it plan multiple steps ahead than a normal version.
 * @return True if current plan need to be update.
 */
int Executive::replan()
{
    if (!dynamic_path_change)
        return 0;
    int checkindex = index + max_check;
    int postindex = (index) ? index - 1 : 0;
    checkindex = (checkindex > path.size()) ? path.size() : checkindex;

    int cost = path[checkindex - 1].tempc - path[postindex].tempc, new_cost = 0;
    State p = start;
    for (int i = index; i < checkindex; i++)
    {
        new_cost += get_cost(path[i], p);
        p = path[i];
    }

    // cerr << new_cost << " " << cost << endl;
    return new_cost > cost * threshold_factor;
}

/**
 * Update the world to current state and accumulate the cost of the agent.
 * Include the agent, dynamic obstacles and weather or not need to replan
 */
void Executive::update_state()
{
    if (plan)
    {
        initialize(0);
    }
    int step = 1;
    int cost = 0;

    while (termination)
    {
        bool collide = false;
        mtx.lock();

        if (path.size() > index)
            start = path[index++];

        for (DynamicObstacle &dynamic : dynamicObstacles)
        {
            update_dynamic(dynamic);
            if (static_cast<int>(dynamic.x) == start.x && static_cast<int>(dynamic.y) == start.y)
                collide = true;
        }

        if (collide)
            cost += 200;

        if (start.x != goal.x || start.y != goal.y)
        {
            cost += 1;
        }

        mtx.unlock();
        unordered_set<State> s = getSTATE();
        if (path.size() <= index)
        {

            clock_t begin = clock();
            update(1);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            cout << elapsed_secs << " SECS " << step << " cost " << cost << endl;
        }
        else if (mode_dynamic && replan() && mode != -1)
        {
            dynamic_lookahead = 1;
            int erase = path.size() - index;
            path.erase(path.end() - erase, path.end());
            planner->setLookAhead(node_per_step);
            clock_t begin = clock();
            update(1);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            cout << elapsed_secs << " SECS " << step << " cost " << cost << endl;
        }
        else
        {
            cout << "Step " << step << " cost " << cost << endl;
        }

        step += 1;

        this_thread::sleep_for(std::chrono::milliseconds(updaterate));
    }
}


/**
 * Update the world to current state and accumulate the cost of the agent.
 * Include the agent, dynamic obstacles and weather or not need to replan
 * Only for automatic performance comparison use
 * @param Maxstep maximum iteration a plan allowed to have
 * @param LookAhead number of iteration a plan allowed to have
 */
int Executive::update_state(int Maxstep, int LookAhead)
{
    mode_dynamic = false;
    path.clear();
    index = 0;
    int cost = 0;
    int step = 1;
    max_check = (max_check > Maxstep) ? Maxstep : max_check;

    if (mode == -1)
        LookAhead = Maxstep;

    initialize(LookAhead);
    planner->max_time = Maxstep;

    while (Maxstep > step++)
    {
        bool collide = false;
        // cerr << " size " << path.size() << " index " << index<< endl;
        if (path.size() > index)
        {
            start = path[index++];
        }

        for (DynamicObstacle &dynamic : dynamicObstacles)
        {
            update_dynamic(dynamic);
            if (static_cast<int>(dynamic.x) == start.x && static_cast<int>(dynamic.y) == start.y)
                collide = true;
        }

        if (collide)
            cost += 200;

        if (start.x != goal.x || start.y != goal.y)
        {
            cost += 1;
        }

        if (path.size() <= index)
            update(0);
        else if (mode_dynamic && replan() && mode != -1)
        {
            dynamic_lookahead = 1;
            int erase = path.size() - index;
            path.erase(path.end() - erase, path.end());
            planner->setLookAhead(node_per_step);
            update(0);
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

/**
 * Helping function for schedulling the update in a fix frequency.
 */
void Executive::scheduledChanged()
{
    threads.push_back(thread([=] { update_state(); }));
}

/**
 * Helping function for terminating the update.
 */
void Executive::terminate()
{
    termination = 0;
    if (threads.size())
        threads[0].join();
}


/**
 * Helping function for simulator to get the info of dynamic obstacles.
 */
vector<DynamicObstacle> Executive::getDynamicObstacle()
{
    vector<DynamicObstacle> temp;
    //mtx.lock();
    for (DynamicObstacle &dynamic : dynamicObstacles)
        temp.push_back(dynamic);
    //mtx.unlock();
    return temp;
}

/**
 * Helping function for simulator to get the info of static obstacles.
 */
unordered_set<StaticObstacle> Executive::getStaticObstacle()
{
    return staticObstacles;
}

/**
 * Helping function for simulator to get the start location of the agent.
 */
State Executive::getStatePoint()
{
    return start;
}

/**
 * Helping function for simulator/controller to set the start location of the agent.
 */
void Executive::setStartPoint(double x, double y)
{
    start = State(x, y, 0);
}

/**
 * Helping function for simulator/controller to set the goal location.
 */
void Executive::setGoalPoint(double x, double y)
{
    goal = State(x, y, 0);
}
