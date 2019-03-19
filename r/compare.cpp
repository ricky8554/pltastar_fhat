
#include "pltastar.h"
#include <iomanip>
#include <algorithm>
using namespace std;

#define LOOKAHEAD 10
#define Collision_cost 200

unsigned int compare(State_P &s1, State_P &s2)
{
    return s1.f() < s2.f();
};

unsigned int compare1(State_P &s1, State_P &s2)
{
    return s1.h_s < s2.h_s;
};

unsigned int compare2(State_P &s1, State_P &s2)
{
    return s1.h_d < s2.h_d;
};

unsigned int compare_static(State_P_Static &s1, State_P_Static &s2)
{
    return s1.h_s < s2.h_s;
};

double PLTASTAR::cost(State_P &s, State_P &s1)
{
    //return s - s1;
    return 1; // all action has same cost;
}

int PLTASTAR::cost_d(State_P &s)
{
    for (DynamicObstacle dynamic : dynamicObstacles)
    {
        
        int x = dynamic.x, y = dynamic.y, t = s.time - startTime, direction = -1;
        if (dynamic.angle == 0)
            direction = 1;
        while (t--)
        {
            for (StaticObstacle staticobs : staticObstacles)
            {
                if ((staticobs.x[0] <= x + direction && x + direction < staticobs.x[1]) && (staticobs.y[1] <= y && y < staticobs.y[2]))
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

int PLTASTAR::ASTAR()
{
    State_P state;
    //recover
    auto it = expandState.begin();
    auto tempit = it;

    while (it != expandState.end())
    {
        tempit = it;
        ++it;
        if (tempit->time < start.time && !(tempit->x == goal.x && tempit->y == goal.y))
            expandState.erase(tempit);
        else
        {
            tempit->h_s = htable_static[getindex(tempit->x, tempit->y)];
            tempit->cost_s = DBL_MAX;
            tempit->cost_d = DBL_MAX;
            if(tempit->h_d > 100)
               tempit->h_d -= 100; 
            else
               tempit->h_d = 0;
        }
        
    }

    //clear open close & open check
    open.clear();
    open.setUpCompare(&compare);
    opencheck.clear();
    close.clear();

    start.cost_s = 0;
    start.cost_d = 0;

    int expansions = 0;
    //push start into open open check & expand state
    open.push(start);
    opencheck.insert(start);

    auto it1 = expandState.find(start);
    if (it1 == expandState.end())
        expandState.insert(start);
    else
    {
        it1->cost_s = 0;
        it1->cost_d = 0;
        start.h_s = htable_static[getindex(it1->x, it1->y)];
    }
    

    do
    {

        state = open.pop();
        opencheck.erase(state);
        close.insert(state);
        expansions += 1;
        cout << "Pick: " << state.x << " " << state.y << " " << state.time << " " << state.h_d << " " << state.h_s << " "<< state.cost_s << " "<< state.cost_d<< endl;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                
                if (!(i || j))
                    continue;

                State_P s(state.x + i, state.y + j, state.time + 1);

                if (close.find(s) == close.end() && checkValid(s))
                {

                    if (goal.x == s.x && goal.y == s.y && cost_d(s) < 500) // consider not update the goal if the value is 1000
                    {
                        auto itt = expandState.find(goal);
                        goal.time = state.time + 1;
                        itt->cost_s = DBL_MAX;
                        itt->time = state.time + 1;
                        s = *itt;
                        expandState.erase(itt);
                        expandState.insert(s);
                    }
                    else if (expandState.find(s) != expandState.end())
                    {
                        s = *expandState.find(s);
                    }
                    else
                    {
                        s.h_s = htable_static[getindex(s.x, s.y)];
                        s.h_d = 0;
                        s.cost_s = DBL_MAX;
                        s.cost_d = DBL_MAX;
                        s.time = state.time + 1;
                        expandState.insert(s);
                    }
                    if (s.cost_s + s.cost_d > state.cost_s + cost(s, state) + state.cost_d + cost_d(s))
                    {

                        s.cost_s = state.cost_s + cost(s, state);
                        s.cost_d = state.cost_d + cost_d(s);
                        expandState.find(s)->cost_s = s.cost_s;
                        expandState.find(s)->cost_d = s.cost_d;
                        parent[s] = state;

                        if (close.find(s) == close.end())
                        {
                            if (opencheck.find(s) == opencheck.end())
                            {
                                //cout << "INSERT " << s.x << " " << s.y << " " << s.time << " " << s.cost << endl;
                                opencheck.insert(s);
                                open.push(s);
                            }
                            else
                            {

                                for (int i = 0; i < open.size(); i++)
                                {
                                    if (open[i] == s)
                                    {
                                        open[i].cost_d = s.cost_d;
                                        open[i].cost_s = s.cost_s;
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
        goal = *expandState.find(goal);
    } while (goal.cost_s + goal.cost_d > state.f() && expansions < LOOKAHEAD); //check collision for goal

    return 0;
}

int PLTASTAR::Learning_Static(PQueue<State_P> open1, unordered_set<State_P> opencheck1, unordered_set<State_P> close1)
{
    unordered_set<State_P_Static> close;
    unordered_set<State_P_Static> opencheck;
    PQueue<State_P_Static> open;

    open.setUpCompare(&compare_static);
    for (auto it = close1.begin(); it != close1.end(); ++it)
    {
        State_P_Static s = State_P_Static(*it);
        if(close.find(s) == close.end())
        {
            close.insert(s);
            htable_static[getindex(it->x, it->y)] = DBL_MAX;
        }
    }

    for (auto it = close1.begin(); it != close1.end(); ++it)
    {
        close.insert(State_P_Static(*it));
        htable_static[getindex(it->x, it->y)] = DBL_MAX;
    }

    for(int i = 0; i< open1.size(); i++)
    {
        if()
    }
    
    while (!close.empty() && !open.empty())
    {
        
        
        State_P_Static s1 = State_P_Static(open.pop());
        opencheck.erase(s1);
        cout << "Static: " << s1.x << "\t" << s1.y << "\t" << s1.cost_s << "\t" << s1.h_s << "\t" << s1.cost_d << "\t" << s1.cost_s << "\t" << s1.f() << endl;

        if (close.find(s1) != close.end())
            close.erase(s1);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (!(i || j))
                    continue;
                
                int t = -1;
                State_P st(s1.x + i, s1.y + j, s1.time + t);
                auto it = close.find(st);
                if (it != close.end())
                {
                    State_P ts = *expandState.find(State_P(it->x, it->y, it->time));
                    if (htable_static[getindex(it->x, it->y)] > cost(ts, s1) + s1.h_s)
                    {
                        ts.h_s = cost(ts, s1) + s1.h_s;
                        htable_static[getindex(it->x, it->y)] = ts.h_s;
                        expandState.find(ts)->h_s = ts.h_s;

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
                                    open[i].h_s = ts.h_s;
                                    //open.moveUP(i);
                                    break;
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

int PLTASTAR::Learning_Dynamic()
{

    for (auto it = close.begin(); it != close.end(); ++it)
    {
        expandState.find(State_P(it->x, it->y, it->time))->h_d = DBL_MAX;
    }
    open.setUpCompare(&compare2);
    while (!close.empty() && !open.empty())
    {
        State_P s1 = open.pop();
        opencheck.erase(s1);

        

        if (close.find(s1) != close.end())
            close.erase(s1);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (!(i || j))
                    continue;
                
                for (int k = 0; k < 2; k++)
                {
                    int t;
                    if (!k)
                        t = -1;
                    else
                        t = 1;
                    State_P st(s1.x + i, s1.y + j, s1.time + t);
                    auto it = close.find(st);
                    if (it != close.end())
                    {
                        State_P ts = *expandState.find(State_P(it->x, it->y, it->time));
                        if (ts.h_d > cost_d(ts) + s1.h_d)
                        {
                            ts.h_d = cost_d(ts) + s1.h_d;
                            expandState.find(ts)->h_d = ts.h_d;

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
                                        open[i].h_d = ts.h_d;
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

State_P PLTASTAR::pickBest()
{
    // if (opencheck.find(goal) != opencheck.end())
    // {
    //     State_P s = State_P(goal.x, goal.y, DBL_MAX, DBL_MAX, goal.h_s, goal.h_d);
    //     s.time = INT_MAX;
    //     expandState.insert(s);
        
    //     return goal;
    // }
    double maxh = open.top().f();
    while (open.top().f() <= maxh)
    {
        goalQ.push(open.pop());
    }

    State_P s = goalQ.top();
    while (!goalQ.empty())
    {
        open.push(goalQ.pop());
    }

    return s;
}

int PLTASTAR::plta_star(State_P star)
{
    State_P s, sgoal;
    if(star.time != 0)
    {
        startTime = star.time;
        start = star;
    }
    path = vector<State_P>();
     cout << "ASTAR" << endl;
    ASTAR();
     cout << "ASTAR END" << endl;
    if (open.empty())
        return 0;
    s = sgoal = pickBest();
    cout << "CHOSE: " << sgoal.x << "\t" << sgoal.y << "\t" << sgoal.cost_s << "\t" << sgoal.h_s << "\t" << sgoal.cost_d << "\t" << sgoal.cost_s << "\t" << sgoal.f() << endl;
    // cout << "STATIC " << open.size() << " " << opencheck.size() << " " <<close.size()<< endl;
    Learning_Static(open,opencheck,close);
    // cout << "DYNAMIC " << open.size() << " " << opencheck.size() << " " <<close.size()<< endl;
    if(htable_static[getindex(90,90)] != 0)
        {
            cout << "ERROR " << htable_static[getindex(90,90)] << endl;
            exit(1);
        }
    Learning_Dynamic();
    
    while (parent.find(s) != parent.end())
    {
        path.push_back(s);
        s = parent[s];
    }
    start = sgoal;
    parent.clear();
    // cout << "START " << endl;
    // for(State s:expandState)
    //     cout << "REMAIN: " << s.x << "\t" << s.y << "\t" << s.cost_s << "\t" << s.h_s << "\t" << s.cost_d << "\t" << s.cost_s << "\t" << s.f() << "\t" <<s.time << endl;
    // cout << "END " << endl;
    return !(start == goal);
}

vector<State_P> PLTASTAR::getPath()
{
    return path;
}

void PLTASTAR::setStartGoal(State_P s, State_P g, int bordw, int bordh)
{
    start = s;
    goal = g;
    goal.time = INT_MAX;
    startTime = 0;
    goalQ.setUpCompare(&compare1);
    boardw = bordw;
    boardh = bordh;
    constructHtable();
    start.h_s = h_value(start);
    goal.h_s = h_value(start);
    expandState.insert(goal);
    expandState.insert(start);
}

void PLTASTAR::setStatic(vector<StaticObstacle> &s)
{
    staticObstacles = s;
}

void PLTASTAR::setDynamic(vector<DynamicObstacle> &d)
{
    dynamicObstacles = d;
}

bool PLTASTAR::ccheck(int x, int y)
{
    if (x < 0 || x >= boardw || y < 0 || y >= boardh)
        return 0;

    for (StaticObstacle staticobs : staticObstacles)
    {
        if ((staticobs.x[0] <= x && x < staticobs.x[1]) && (staticobs.y[1] <= y && y < staticobs.y[2]))
            return 0;
    }
    return 1;
}

int PLTASTAR::getindex(int x, int y)
{
    return boardw * y + x;
}

void PLTASTAR ::constructHtable()
{
    clock_t begin = clock();
    htable = new float[boardw * boardh];
    for (int i = 0; i < boardw * boardh; i++)
    {
        htable[i] = FLT_MAX;
    }
    htable[getindex(goal.x, goal.y)] = 0;
    PQueue<State_P> q(&compare1);
    State_P s = State_P(goal.x, goal.y);
    s.h_s = 0;
    q.push(s);
    while (!q.empty())
    {
        State_P ss = q.pop();
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (!(i || j))
                    continue;

                State_P news(ss.x + i, ss.y + j);
                news.h_s = cost(ss, news) + htable[getindex(ss.x, ss.y)];
                if (ccheck(news.x, news.y) && news.h_s < htable[getindex(news.x, news.y)])
                {
                    htable[getindex(news.x, news.y)] = news.h_s;
                    q.push(news);
                }
            }
        }
    }
    htable_static = new float[boardw * boardh];
    memcpy(htable_static, htable, sizeof(float) * boardw * boardh);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "INITIAL HTABLE " << elapsed_secs << " SECS" << endl;

    // for(int i = 0; i< boardw;i++)
    // {
    //     for(int j = 0; j< boardh;j++)
    //         if(htable[getindex(i, j)] > 999)
    //         {
    //             cerr << setprecision(3) << "999" << " ";
    //         }
    //         else if(htable[getindex(i, j)] < 10)
    //         {
    //             cerr << setprecision(3) << "00" << htable[getindex(i, j)] << " ";
    //         }
    //         else if(htable[getindex(i, j)] < 100)
    //         {
    //             cerr << setprecision(3) << "0" << htable[getindex(i, j)] << " ";
    //         }
    //         else
    //             cerr << setprecision(3) << htable[getindex(i, j)] << " ";

    //     cerr << endl;
    // }
}

int PLTASTAR::checkValid(State_P &s)
{

    if (s.x < 0 || s.x >= boardw || s.y < 0 || s.y >= boardh)
        return 0;

    for (StaticObstacle staticobs : staticObstacles)
    {
        if ((staticobs.x[0] <= s.x && s.x < staticobs.x[1]) && (staticobs.y[1] <= s.y && s.y < staticobs.y[2]))
            return 0;
    }

    return 1;
}
double PLTASTAR::h_value(State_P &s)
{
    return htable[getindex(s.x, s.y)];
}

the static for the goal point will be changed 