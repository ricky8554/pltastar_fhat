#ifndef __PLAN_H__
#define __PLAN_H__

#include <iostream>
#include <unordered_set>
#include <vector>
#include "state.h"
#include <queue>
#include <cfloat>
#include <iomanip>

using namespace std;
static const double Epsilon = std::numeric_limits<double>::epsilon();
static const double Threshold = sqrt(Epsilon);

class Plan
{
  protected:
    int LOOKAHEAD = 1000, Collision_cost = 200, decay = 20; //7 8 36
    double std = 1, stdi = 0.1;
    int boardw, boardh, startTime, startDynamic = 0, movingSteps = 1;
    double sqrt2 = 1.41421356237, check_interval = 0.05;
    float *htable;
    State goal_plan;
    State start_plan;
    DynamicObstacle dummy_dynamic_obs;

    vector<DynamicObstacle> dynamicObstacles;
    unordered_set<DynamicObstacle> dynamicObstacles_map;
    unordered_set<StaticObstacle> staticObstacles;
    StaticObstacle dummy_static_obs;
    vector<State> path;

    unordered_set<point> debug;
    unordered_set<point> debug1;
    State debugstart = State(-1, -1);

    //pltastar
    float *htable_static;
    //fhat
    float *dtable, *derrtable;

  public:
    int max_time = 0;
    // Default constructor
    Plan()
    {
        ios_base::sync_with_stdio(false);
    };

    virtual ~Plan(){};

    void setLookAhead(int look)
    {
        LOOKAHEAD = look;
    };

    virtual void setStartGoal(State starta, State goala, int bordw, int bordh)
    {
        cerr << "ERROR setStartGoal CALL SUPER" << endl;
    };

    virtual int plan(State start)
    {
        cerr << "ERROR plan CALL SUPER" << endl;
        return 0;
    };

    virtual void setStatic(unordered_set<StaticObstacle> &s)
    {
        cerr << "ERROR setStatic CALL SUPER" << endl;
    };

    void set_time(int time)
    {
        startTime = time;
    }

    // void update_dynamic(DynamicObstacle &d, int t)
    // {
    //     int x;
    //     for (int i = 0; i < 2; i++)
    //     {
    //         x = d.x + cos(d.angle);
    //         dummy_static_obs.set(x, d.y);
    //         if (staticObstacles.find(dummy_static_obs) == staticObstacles.end() && x >= 0 && x < boardw)
    //         {
    //             d.x = x;
    //             d.t = t;
    //             return;
    //         }
    //         else
    //             d.angle = (d.angle) ? 0 : 3.14159265;
    //     }
    // }

    void setDynamic(vector<DynamicObstacle> &d)
    {
        dynamicObstacles = d;
    };

    virtual unordered_set<State> getSTATE()
    {
        cerr << "ERROR getSTATE CALL SUPER" << endl;
        return unordered_set<State>();
    };

    vector<State> getPath()
    {
        return path;
    };

    class Compare
    {
      public:
        bool operator()(State s, State s1)
        {
            return s.fakeh > s1.fakeh;
        }
    };

    //FHAT

    double getDistance(int x, int y)
    {
        int a = abs(x - goal_plan.x), b = abs(y - goal_plan.y);
        return (a > b) ? a : b;
    }

    double getDistance(State *s)
    {
        return getDistance(s->x, s->y);
    }

    //PLTASTAR
    double h_value_static(State *s)
    {
        return htable_static[getindex(s->x, s->y)];
    }

    //SHARE

    double h_value(State *s)
    {
        return htable[getindex(s->x, s->y)];
    }

    void create_pred(State *s, int t)
    {
        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
                s->pred.emplace(s->x + i, s->y + j, t);
    }

    int getindex(int x, int y)
    {
        return boardw * y + x;
    }

    double get_probability(DynamicObstacle &dynamic, double x, double y, double t)
    {
        double dhead = dynamic.estimate_h, dspeed = dynamic.estimate_s;
        double mx = cos(dhead) * dspeed * t + dynamic.x, my = sin(dhead) * dspeed * t + dynamic.y;
        double stderr = std + stdi * t;
        double variance = stderr * stderr;
        double dx = x-mx,dy = y-my;
        // double pxy = ((x - mx) * (y - my)) / variance;
        double first = 1 / (2.0 * M_PI * variance );
        double second = -1 / 2.0;
        double third = ((dx*dx + dy*dy)) / variance;
        return first * exp(second* third);
    }

    int cost_d(State *ps, State *s)
    {
        int factor1;
        double timeinterval = factor1 = 10;
        int cx = ps->x, cy = ps->y, fx = s->x, fy = s->y, dt = s->time - startTime;
        double diffx = (fx - cx) / timeinterval, diffy = (fy - cy) / timeinterval;
        double p_collide = 0;
        for (int i = 1; i <= factor1; i++)
        {
            double x = cx + diffx * i, y = cy + diffy * i;
            double temp_p_collide = 0, p_not_collide = 1;

            for (DynamicObstacle &dynamic : dynamicObstacles)
                p_not_collide *= (1 - get_probability(dynamic,x,y,(dt - 1 + i / timeinterval)));

            temp_p_collide = 1 - p_not_collide;

            if (temp_p_collide > p_collide)
                p_collide = temp_p_collide;
        }
        if(p_collide > 1)
            p_collide = 1;
        return p_collide * Collision_cost;
    }

    int checkValid(State *s)
    {
        if (s->x < 0 || s->x >= boardw || s->y < 0 || s->y >= boardh)
            return 0;
        dummy_static_obs.set(s->x, s->y);
        if (staticObstacles.find(dummy_static_obs) != staticObstacles.end())
            return 0;

        return 1;
    }

    //remove cost
    double cost(State &s, State &s1)
    {
        if (s.x == goal_plan.x && s1.x == goal_plan.x && s.y == goal_plan.y && s1.y == goal_plan.y)
            return 0;
        // if(s1.x == goal_plan.x && s1.y == goal_plan.y)
        //     return 0;
        return 1;
    }

    double cost(State *s, State *s1)
    {
        if (s->x == goal_plan.x && s1->x == goal_plan.x && s->y == goal_plan.y && s1->y == goal_plan.y)
            return 0;
        // if(s1->x == goal_plan.x && s1->y == goal_plan.y)
        //     return 0;
        return 1;
    }

    bool isGoal(State *s)
    {
        return (s->x == goal_plan.x && s->y == goal_plan.y);
    }

    bool ccheck(int x, int y)
    {
        dummy_static_obs.set(x, y);
        if ((x < 0 || x >= boardw || y < 0 || y >= boardh) || staticObstacles.find(dummy_static_obs) != staticObstacles.end())
            return 0;
        return 1;
    }

    void constructHtable(State start, State goal)
    {
        goal_plan = goal;
        start_plan = start;
        std::priority_queue<State, std::vector<State>, Compare> q;

        int boardsize = boardw * boardh;
        htable = new float[boardsize];
        dtable = new float[boardsize];
        derrtable = new float[boardsize];
        bool checkTable[boardsize];
        for (int i = 0; i < boardsize; i++)
        {
            htable[i] = FLT_MAX;
            checkTable[i] = false;
        }

        htable[getindex(goal.x, goal.y)] = goal.fakeh = 0;

        q.push(goal);
        while (!q.empty())
        {
            State state = q.top();
            q.pop();
            int index = getindex(state.x, state.y);
            checkTable[index] = true;
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    int newindex = getindex(state.x + i, state.y + j);
                    if (checkTable[newindex])
                        continue;
                    State newstate = State(state.x + i, state.y + j);
                    newstate.fakeh = cost(state, newstate) + htable[index];
                    if (ccheck(newstate.x, newstate.y) && newstate.fakeh < htable[newindex])
                    {
                        htable[newindex] = newstate.fakeh;
                        q.push(newstate);
                    }
                }
            }
        }
        for (int i = 0; i < boardsize; i++)
        {
            int x = i % boardw, y = i / boardw;

            if (ccheck(x, y))
            {
                dtable[i] = derrtable[i] = htable[i] = getDistance(x, y);
                // float a = abs(x - goal.x), b = abs(y - goal.y);  //H_DIS
                // htable[i] = (a > b) ? a : b;                     //H_DIS
            }
            else
                htable[i] = dtable[i] = derrtable[i] = FLT_MAX;
        }

        htable_static = new float[boardw * boardh];
        memcpy(htable_static, htable, sizeof(float) * boardw * boardh);
    }

    void print_table_helper(int x, int y, float *table)
    {
        std::cerr << std::setprecision(0) << std::fixed;
        for (int i = boardh - 1; i >= 0; i--)
        {
            for (int j = 0; j < boardw; j++)
            {
                if (x == j && y == i)
                    cerr << "\033[0;31m";
                else if (debugstart.x == j && debugstart.y == i)
                    cerr << "\033[0;34m";
                else if (debug.find(point(j, i)) != debug.end())
                    cerr << "\033[0;32m";
                else if (debug1.find(point(j, i)) != debug.end())
                    cerr << "\033[0;33m";
                else
                    cerr << "\033[0;30m";
                int index = getindex(j, i);
                if (table[index] > 1000)
                    cerr << "-01 ";
                else if (table[index] < 10)
                    cerr << "00" << table[index] << " ";
                else if (table[index] < 100)
                    cerr << 0 << table[index] << " ";
                else
                    cerr << table[index] << " ";
            }
            cerr << endl;
        }
        cerr << "\033[0;30m";
        std::cerr << std::setprecision(2) << std::fixed;
    }

    void printHtable(int x, int y)
    {
        print_table_helper(x, y, htable);
    }

    void printHStable(int x, int y)
    {
        print_table_helper(x, y, htable_static);
    }

    void printDtable(int x, int y)
    {
        print_table_helper(x, y, dtable);
    }

    void printDerrtable(int x, int y)
    {
        print_table_helper(x, y, derrtable);
    }

    // void PLTASTAR_FHAT::constructHtable()
    // {
    //     // clock_t begin = clock();
    //     htable = new float[boardw * boardh];
    //     dtable = new double[boardw * boardh];
    //     derrtable = new double[boardw * boardh];
    //     for (int i = 0; i < boardw * boardh; i++)
    //     {
    //         htable[i] = FLT_MAX;
    //     }
    //     htable[getindex(goal->x, goal->y)] = 0;

    //     for (int i = 0; i < boardh; i++)
    //     {
    //         for (int j = 0; j < boardw; j++)
    //         {
    //             int index = getindex(j, i);
    //             float a = abs(j - goal->x), b = abs(i - goal->y);
    //             if (ccheck(x, y))
    //             {
    //                 htable[index] = (a > b) ? a : b;
    //                 dtable[i] = derrtable[i] = getDistance(x, y);
    //             }
    //             else
    //             {
    //                 dtable[i] = derrtable[i] = htable[index] = FLT_MAX;
    //             }
    //             htable[index] = (a > b) ? a : b;
    //             dtable[index] = derrtable[index] = getDistance(j, i);
    //         }
    //     }
    //     htable_static = new float[boardw * boardh];
    //     memcpy(htable_static, htable, sizeof(float) * boardw * boardh);
    //     // clock_t end = clock();
    //     // double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    //     //cout << "INITIAL HTABLE " << elapsed_secs << " SECS" << endl;
    // }
};

#endif