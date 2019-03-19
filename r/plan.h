#ifndef __PLAN_H__
#define __PLAN_H__

#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include "state.h"
#include <queue>
#include <cfloat>
#include <iomanip>

using namespace std;

class Plan
{
  protected:
    int LOOKAHEAD = 10, Collision_cost = 200, decay = 20; //7 8
    int boardw, boardh, startTime;
    double sqrt2 = 1.41421356237;
    float *htable_base;
    State goal_plan;
    State start_plan;
    point dummy_point;

    vector<DynamicObstacle> dynamicObstacles;
    unordered_set<StaticObstacle> staticObstacles;
    StaticObstacle dummy_static_obs;
    vector<State> path;

    unordered_set<point> debug;
    unordered_set<point> debug1;
    State debugstart = State(-1, -1, -1, -1);

    // //pltastar
    //double *htable_staticbase;
    // //fhat
    double *dtablebase, *derrtablebase;
    unordered_map<point, double> htable_static;
    unordered_map<point, double> dtable, derrtable;

  public:
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

    virtual void setDynamic(vector<DynamicObstacle> &d)
    {
        cerr << "ERROR setDynamic CALL SUPER" << endl;
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
        dummy_point.set(s);
        if (htable_static.find(dummy_point) == htable_static.end())
            htable_static[dummy_point] = htable_base[getindex(dummy_point.x, dummy_point.y)];
        return htable_static[dummy_point];
    }

    //SHARE

    double h_value(State *s)
    {
        return htable_base[getindex(s->x, s->y)];
    }

    // void create_pred(State *s, int t)
    // {
    //     for (int i = -1; i <= 1; i++)
    //         for (int j = -1; j <= 1; j++)
    //             s->pred.emplace(s->x + i, s->y + j, t);
    // }

    int getindex(int x, int y)
    {
        return boardw * y + x;
    }

    double cost_d(State *ps, State *s)
    {
        double timeinterval = 20;
        int cx = ps->x, cy = ps->y, dt = s->time - startTime;
        double dx = (s->x - cx) / timeinterval, dy = (s->y - cy) / timeinterval;
        //double tcost = 0;
        double p_collide = 0;
        for (int i = 1; i <= timeinterval; i++)
        {
            double x = cx + dx * i, y = cy + dy * i;
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

        return p_collide * Collision_cost;
    }

    int checkValid2(double x, double y)
    {
        if (x < 0 || x >= boardw || y < 0 || y >= boardh)
            return 0;
        dummy_static_obs.set(round(x), round(y));
        if (staticObstacles.find(dummy_static_obs) != staticObstacles.end())
            return 0;
        return 1;
    }

    int checkValid(State *ps, State *s)
    {
        double timeinterval = 20;
        int cx = ps->x, cy = ps->y;
        double dx = (s->x - cx) / timeinterval, dy = (s->y - cy) / timeinterval;
        for (int i = 1; i <= timeinterval; i++)
        {
            double x = cx + dx * i, y = cy + dy * i;

            if (x < 0 || x >= boardw || y < 0 || y >= boardh)
                return 0;
            // cerr << "testing " << x <<  " " << y << " "<<round(x) << " " << round(y) << endl;
            dummy_static_obs.set(round(x), round(y));
            if (staticObstacles.find(dummy_static_obs) != staticObstacles.end())
                return 0;
        }
        return 1;
    }

    //remove cost
    double cost(State &s, State &s1)
    {
        if (s.x == goal_plan.x && s1.x == goal_plan.x && s.y == goal_plan.y && s1.y == goal_plan.y)
            return 0;
        return 1;
    }

    double cost(State *s, State *s1)
    {
        if (s->x == goal_plan.x && s1->x == goal_plan.x && s->y == goal_plan.y && s1->y == goal_plan.y)
            return 0;
        return 1;
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
        htable_base = new float[boardsize];
        dtablebase = new double[boardsize];
        derrtablebase = new double[boardsize];
        bool checkTable[boardsize];
        for (int i = 0; i < boardsize; i++)
        {
            htable_base[i] = FLT_MAX;
            checkTable[i] = false;
        }

        // htable[getindex(goal.x, goal.y)] = goal.fakeh = 0;

        // q.push(goal);
        // while (!q.empty())
        // {
        //     State state = q.top();
        //     q.pop();
        //     int index = getindex(state.x, state.y);
        //     checkTable[index] = true;
        //     for (int i = -1; i <= 1; i++)
        //     {
        //         for (int j = -1; j <= 1; j++)
        //         {
        //             int newindex = getindex(state.x + i, state.y + j);
        //             if (checkTable[newindex])
        //                 continue;
        //             State newstate = State(state.x + i, state.y + j);
        //             newstate.fakeh = cost(state, newstate) + htable[index];
        //             if (ccheck(newstate.x, newstate.y) && newstate.fakeh < htable[newindex])
        //             {
        //                 htable[newindex] = newstate.fakeh;
        //                 q.push(newstate);
        //             }
        //         }
        //     }
        // }
        for (int i = 0; i < boardsize; i++)
        {
            int x = i % boardw, y = i / boardw;

            if (ccheck(x, y))
            {
                dtablebase[i] = derrtablebase[i] = htable_base[i] = getDistance(x, y);
            }
            else
                htable_base[i] = dtablebase[i] = derrtablebase[i] = FLT_MAX;
        }
    }

    // void printHtable(int x, int y)
    // {
    //     std::cerr << std::setprecision(0) << std::fixed;
    //     for (int i = boardh - 1; i >= 0; i--)
    //     {
    //         for (int j = 0; j < boardw; j++)
    //         {
    //             if(x == j && y == i)
    //                 cerr << "\033[0;31m";
    //             else if(debugstart.x == j && debugstart.y == i)
    //                 cerr << "\033[0;34m";
    //             else if(debug.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;32m";
    //             else if(debug1.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;33m";
    //             else
    //                 cerr << "\033[0;30m";
    //             int index = getindex(j, i);
    //             if (htable[index] > 1000)
    //             {
    //                 cerr << "-01 ";
    //             }
    //             else if (htable[index] < 10)
    //             {
    //                 cerr << "00" << htable[index] << " ";
    //             }
    //             else if (htable[index] < 100)
    //             {
    //                 cerr << 0 << htable[index] << " ";
    //             }
    //             else
    //             {
    //                 cerr << htable[index] << " ";
    //             }
    //         }
    //         cerr << endl;
    //     }
    //     cerr << "\033[0;30m";

    //     std::cerr << std::setprecision(2) << std::fixed;
    // }

    // void printHStable(int x, int y)
    // {
    //     std::cerr << std::setprecision(0) << std::fixed;
    //     for (int i = boardh - 1; i >= 0; i--)
    //     {
    //         for (int j = 0; j < boardw; j++)
    //         {
    //             if(x == j && y == i)
    //                 cerr << "\033[0;31m";
    //             else if(debugstart.x == j && debugstart.y == i)
    //                 cerr << "\033[0;34m";
    //             else if(debug.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;32m";
    //             else if(debug1.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;33m";
    //             else
    //                 cerr << "\033[0;30m";
    //             int index = getindex(j, i);
    //             if (htable_static[index] > 1000)
    //             {
    //                 cerr << "-01 ";
    //             }
    //             else if (htable_static[index] < 10)
    //             {
    //                 cerr << "00" << htable_static[index] << " ";
    //             }
    //             else if (htable_static[index] < 100)
    //             {
    //                 cerr << 0 << htable_static[index] << " ";
    //             }
    //             else
    //             {
    //                 cerr << htable_static[index] << " ";
    //             }
    //         }
    //         cerr << endl;
    //     }
    //     cerr << "\033[0;30m";
    //     std::cerr << std::setprecision(2) << std::fixed;
    // }

    // void printDtable(int x, int y)
    // {
    //     std::cerr << std::setprecision(0) << std::fixed;
    //     for (int i = boardh - 1; i >= 0; i--)
    //     {
    //         for (int j = 0; j < boardw; j++)
    //         {
    //             if(x == j && y == i)
    //                 cerr << "\033[0;31m";
    //             else if(debugstart.x == j && debugstart.y == i)
    //                 cerr << "\033[0;34m";
    //             else if(debug.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;32m";
    //             else if(debug1.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;33m";
    //             else
    //                 cerr << "\033[0;30m";
    //             int index = getindex(j, i);
    //             if (dtable[index] > 1000)
    //             {
    //                 cerr << "-01 ";
    //             }
    //             else if (dtable[index] < 10)
    //             {
    //                 cerr << "00" << dtable[index] << " ";
    //             }
    //             else if (dtable[index] < 100)
    //             {
    //                 cerr << 0 << dtable[index] << " ";
    //             }
    //             else
    //             {
    //                 cerr << dtable[index] << " ";
    //             }
    //         }
    //         cerr << endl;
    //     }
    //     cerr << "\033[0;30m";
    //     std::cerr << std::setprecision(2) << std::fixed;
    // }

    // void printDerrtable(int x, int y)
    // {
    //     std::cerr << std::setprecision(0) << std::fixed;
    //     for (int i = boardh - 1; i >= 0; i--)
    //     {
    //         for (int j = 0; j < boardw; j++)
    //         {
    //             if(x == j && y == i)
    //                 cerr << "\033[0;31m";
    //             else if(debugstart.x == j && debugstart.y == i)
    //                 cerr << "\033[0;34m";
    //             else if(debug.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;32m";
    //             else if(debug1.find(point(j,i)) != debug.end())
    //                 cerr << "\033[0;33m";
    //             else
    //                 cerr << "\033[0;30m";
    //             int index = getindex(j, i);
    //             if (derrtable[index] > 1000)
    //             {
    //                 cerr << "-01"
    //                      << " ";
    //             }
    //             else if (derrtable[index] < 10)
    //             {
    //                 cerr << "00" << derrtable[index] << " ";
    //             }
    //             else if (derrtable[index] < 100)
    //             {
    //                 cerr << 0 << derrtable[index] << " ";
    //             }
    //             else
    //             {
    //                 cerr << derrtable[index] << " ";
    //             }
    //         }
    //         cerr << endl;
    //     }
    //     cerr << "\033[0;30m";
    //     std::cerr << std::setprecision(2) << std::fixed;
    // }

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