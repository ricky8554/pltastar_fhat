
#include "skeloton.h"
#include <iomanip>
#include <algorithm>
using namespace std;

#define LOOKAHEAD 10
#define Collision_cost 200
#define sqrt2 1.41421356237

unsigned int compare(State &s1, State &s2)
{
    return s1.f_hat() < s2.f_hat();
};

unsigned int compare1(State &s1, State &s2)
{
    return s1.h < s2.h; //chage to h_hat?
};

double Skeloton::cost(State &s, State &s1)
{
    //return s - s1;
    if (s.x == goal.x && s1.x == goal.x && s.y == goal.y && s1.y == goal.y)
        return 0;
    return 1;
}

int Skeloton::cost_obs(State &s, int timestamp)
{
    for (DynamicObstacle dynamic : dynamicObstacles)
    {
        int x = dynamic.x, y = dynamic.y, t = timestamp - startTime, direction = -1;
        if (dynamic.angle == 0)
            direction = 1;
        while (t--)
        {
            for (StaticObstacle staticobs : staticObstacles)
            {
                if (staticobs.x == x + direction && staticobs.y == y)
                {
                    if (direction == 1)
                        direction = -1;
                    else
                        direction = 1;

                    break;
                }
            }
            if (x + direction < 0 || x + direction >= boardw || y < 0 || y >= boardh)
            {
                if (direction == 1)
                    direction = -1;
                else
                    direction = 1;
            }
            x += direction;
        }

        if (x == s.x && y == s.y)
        {
            return Collision_cost;
        }
    }
    return 0;
}

int Skeloton::checkValid(State &s, int timestamp)
{

    if (s.x < 0 || s.x >= boardw || s.y < 0 || s.y >= boardh)
        return 0;

    for (StaticObstacle staticobs : staticObstacles)
    {
        if (staticobs.x == s.x && staticobs.y == s.y)
            return 0;
    }

    return 1;
}
double Skeloton::h_value(State &s)
{
    return htable[getindex(s.x, s.y)];
}

int Skeloton::getDistance(State &s)
{
    int a = abs(s.x-goal.x), b = abs(s.y-goal.y);
    return (a > b) ? a:b;
}

int Skeloton::ASTAR()
{
    State state;
    //recover

    auto it = expandState.begin();
    auto tempit = it;
    double sum = 0;

    while (it != expandState.end())
    {
        tempit = it;
        ++it;
        if (tempit->time < start.time && !(tempit->x == goal.x && tempit->y == goal.y))
            expandState.erase(tempit);
        else
            tempit->cost = DBL_MAX;
    }

    //clear open close & open check
    open.clear();
    open.setUpCompare(&compare);
    opencheck.clear();
    close.clear();

    start.cost = 0;

    int expansions = 0;
    //push start into open open check & expand state
    open.push(start);
    opencheck.insert(start);

    auto it1 = expandState.find(start);
    if (it1 == expandState.end())
        expandState.insert(start);
    else
    {
        it1->cost = 0;
        start.h = it1->h;
    }

    do
    {

        state = open.pop();
        opencheck.erase(state);
        close.insert(state);
        expansions += 1;
        //cout << "Pick: " << state.x << " " << state.y << " " << state.time << " " << state.h << " " << state.cost << " "<< state.f() << " " << state.f_hat()<< endl;
        double smallest = 1000000000;
       

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                State s(state.x + i, state.y + j, state.time + 1);
                if (checkValid(s, state.time + 1) && close.find(s) == close.end())
                {
                    if (expandState.find(s) != expandState.end())
                    {
                        s = *expandState.find(s);
                    }
                    else
                    {

                        s.h = h_value(s);
                        s.cost = DBL_MAX;
                        s.time = state.time + 1;
                        expandState.insert(s);
                    }

                    if (s.cost > state.cost + cost(state, s) + cost_obs(s, s.time))
                    {
                        s.cost = state.cost + cost(state, s) + cost_obs(s, s.time);
                        expandState.find(s)->cost = s.cost;
                        s.error = error * getDistance(s);
                        expandState.find(s)->h_error = s.error;
                        parent[s] = state;
                        if (s.f() < smallest)
                            smallest = s.f();

                        if (close.find(s) == close.end())
                        {
                            if (opencheck.find(s) == opencheck.end())
                            {
                                //cout << "INSERT " << s.x << " " << s.y << " " << s.time << " " << s.cost << " " << s.h << " " << s.h_error<< endl;
                                opencheck.insert(s);
                                open.push(s);
                            }
                            else
                            {

                                for (int i = 0; i < open.size(); i++)
                                {
                                    if (open[i] == s)
                                    {
                                        open[i].cost = s.cost;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        sum += smallest - state.f();
        cerr << state.f() << " " << smallest << endl;
        open.reBuild();
    } while (expansions < LOOKAHEAD);

    error = sum /LOOKAHEAD;
    cout << "ERROR " << error << endl;
    return 0;
}

int Skeloton::Dijkstra()
{

    for (auto it = close.begin(); it != close.end(); ++it)
    {
        expandState.find(State(it->x, it->y, it->time))->h = DBL_MAX;
    }
    open.setUpCompare(&compare1);

    while (!close.empty())
    {
        if (open.empty())
        {
            for (State s : close)
            {
                cout << s.x << " " << s.y << endl;
            }
        }
        State s1 = open.pop();
        opencheck.erase(s1);

        if (close.find(s1) != close.end())
            close.erase(s1);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {

                for (int k = 0; k < 2; k++)
                {
                    int t;
                    if (!k)
                        t = -1;
                    else
                        t = 1;
                    State st(s1.x + i, s1.y + j, s1.time + t);
                    auto it = close.find(st);
                    if (it != close.end())
                    {
                        State ts = *expandState.find(State(it->x, it->y, it->time));
                        if (ts.h > cost(ts, s1) + s1.h + cost_obs(ts, ts.time))
                        {
                            ts.h = cost(ts, s1) + s1.h + cost_obs(ts, ts.time);
                            expandState.find(ts)->h = ts.h;

                            if (opencheck.find(ts) == opencheck.end())
                            {
                                opencheck.insert(ts);
                                open.push(ts);
                            }
                            else
                            {
                                for (int i = 0; i < open.size(); i++)
                                {
                                    if (open[i] == ts)
                                    {
                                        open[i].h = ts.h;
                                        //open.moveUP(i);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        open.reBuild();
    }
    return 0;
}

State Skeloton::pickBest()
{
    double maxh = open.top().f();
    while (!open.empty() && open.top().f() <= maxh)
    {
        goalQ.push(open.pop());
    }

    open.setUpCompare(&compare1);

    State s = goalQ.top();

    while (!goalQ.empty())
    {
        open.push(goalQ.pop());
    }

    return s;
}

int Skeloton::LSS_LRTASTAR(State star)
{
    startTime = star.time;
    State s, sgoal;
    path = vector<State>();
    if (star.time != 0)
        start = star;

    ASTAR();
    if (open.empty())
        return 0;

    s = sgoal = pickBest();

    cout << "CHOSE: " << sgoal.x << "\t" << sgoal.y << "\t" << sgoal.cost << "\t" << sgoal.h << "\t" << sgoal.f() << endl;
    Dijkstra();
    while (parent.find(s) != parent.end())
    {
        path.push_back(s);
        s = parent[s];
    }
    start = sgoal;
    parent.clear();
    return !(start == goal);
}

vector<State> Skeloton::getPath()
{
    return path;
}

void Skeloton::setStartGoal(State s, State g, int bordw, int bordh)
{
    start = s;
    goal = g;
    goal.time = INT_MAX;
    goalQ.setUpCompare(&compare1);
    boardw = bordw;
    boardh = bordh;
    constructHtable();
    start.h = h_value(start);
    expandState.insert(goal);
    expandState.insert(start);
}

void Skeloton::setStatic(vector<StaticObstacle> &s)
{
    staticObstacles = s;
}

void Skeloton::setDynamic(vector<DynamicObstacle> &d)
{
    dynamicObstacles = d;
}

bool Skeloton::ccheck(int x, int y)
{
    if (x < 0 || x >= boardw || y < 0 || y >= boardh)
        return 0;

    for (StaticObstacle staticobs : staticObstacles)
    {
        if (staticobs.x == x && staticobs.y == y)
            return 0;
    }
    return 1;
}

int Skeloton::getindex(int x, int y)
{
    return boardw * y + x;
}

void Skeloton::constructHtable()
{
    clock_t begin = clock();
    htable = new float[boardw * boardh];
    for (int i = 0; i < boardw * boardh; i++)
    {
        htable[i] = FLT_MAX;
    }
    htable[getindex(goal.x, goal.y)] = 0;
    PQueue<State> q(&compare1);
    State s = State(goal.x, goal.y);
    s.h = 0;
    q.push(s);
    while (!q.empty())
    {
        State ss = q.pop();
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (!(i || j))
                    continue;
                State news(ss.x + i, ss.y + j);
                news.h = cost(ss, news) + htable[getindex(ss.x, ss.y)];
                if (ccheck(news.x, news.y) && news.h < htable[getindex(news.x, news.y)])
                {
                    htable[getindex(news.x, news.y)] = news.h;
                    q.push(news);
                }
            }
        }
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "INITIAL HTABLE " << elapsed_secs << " SECS" << endl;
}
