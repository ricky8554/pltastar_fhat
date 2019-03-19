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

class Plan
{
  protected:
    int LOOKAHEAD = 36, Collision_cost = 200, decay = 20; //7 8 36
    int boardw, boardh, startTime, startDynamic = 0, movingSteps = 1;
    double sqrt2 = 1.41421356237, check_interval = 0.05;
    float *htable_base;
    State goal_plan;
    State start_plan;
    point dummy_point;
    DynamicObstacle dummy_dynamic_obs;

    vector<DynamicObstacle> dynamicObstacles;
    unordered_set<DynamicObstacle> dynamicObstacles_map;
    unordered_set<StaticObstacle> staticObstacles;
    StaticObstacle dummy_static_obs;
    vector<State> path;

    unordered_set<point> debug;
    unordered_set<point> debug1;
    State debugstart = State(-1, -1,-1,-1);

    //pltastar
    // float *htable_static;
    //fhat
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

    double d_value(State *s)
    {
        dummy_point.set(s);
        if (dtable.find(dummy_point) == dtable.end())
            dtable[dummy_point] = getDistance(dummy_point.x, dummy_point.y);
        return dtable[dummy_point];
    }

    void set_d_value(State *s,double value)
    {
        dummy_point.set(s);
        dtable[dummy_point] = value;
    }

    double derr_value(State *s)
    {
        dummy_point.set(s);
        if (derrtable.find(dummy_point) == derrtable.end())
            derrtable[dummy_point] = getDistance(dummy_point.x, dummy_point.y);
        return derrtable[dummy_point];
    }

    void set_derr_value(State *s,double value)
    {
        dummy_point.set(s);
        derrtable[dummy_point] = value;
    }

    //PLTASTAR
    double h_value_static(State *s)
    {
        dummy_point.set(s);
        if (htable_static.find(dummy_point) == htable_static.end())
            htable_static[dummy_point] = htable_base[getindex(dummy_point.x, dummy_point.y)];
        return htable_static[dummy_point];
    }

    void set_h_value_static(State *s,double value)
    {
        dummy_point.set(s);
        htable_static[dummy_point] = value;
    }

    //SHARE

    void find_avaliable_state(State* state, State* dummy, int &dx, int &dy)
    {   
        int cx = state->x, cy = state->y, fx = dummy->x, fy = dummy->y;
        int distance = sqrt((cx - fx) * (cx - fx) + (cy - fy) * (cy - fy));
        int factor1 = int(distance / check_interval);
        double factor = factor1;
        double diffx = (fx - cx) / factor, diffy = (fy - cy) / factor;
        double finalx = cx, finaly = cy;
        for (int i = 0; i < factor1; i++)
        {
            finalx += diffx;
            finaly += diffy;
            if (!checkValid2(finalx,finaly))
            {
                finalx -= diffx;
                finaly -= diffy;
                break;
            }
        }
        dx = dy = 0;
        dummy->set(int(finalx), int(finaly), dx, dy, state->time + 1);
        if(!checkValid2(dummy->x,dummy->y))
            exit(-1);
    }

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

    int cost_d(State *ps, State *s)
    {
        int cx = ps->x, cy = ps->y, fx = s->x, fy = s->y;
        int distance = sqrt((cx - fx) * (cx - fx) + (cy - fy) * (cy - fy));
        int factor1 = int(distance / check_interval);
        double factor = factor1;
        double diffx = (fx - cx) / factor, diffy = (fy - cy) / factor;
        int dt = s->time - startTime;

        // double timeinterval = 20;
        // int cx = ps->x, cy = ps->y, dt = s->time - startTime;
        // double dx = (s->x - cx) / timeinterval, dy = (s->y - cy) / timeinterval;
        //double tcost = 0;
        double p_collide = 0;
        for (int i = 1; i <= factor1; i++)
        {
            double x = cx + diffx * i, y = cy + diffy * i;
            double temp_p_collide = 0, p_not_collide = 1;
            for (DynamicObstacle dynamic : dynamicObstacles)
            {
                double dhead = dynamic.estimate_h, dspeed = dynamic.estimate_s;
                int dyt = (dt > 20) ? 20 : dt;
                double dtime = (dt - 1 + i / factor1);
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

    int checkValid2(int x, int y)
    {  
        if (x < 0 || x >= boardw || y < 0 || y >= boardh)
            return 0;
        dummy_static_obs.set(x ,y);
        
        if (staticObstacles.find(dummy_static_obs) != staticObstacles.end())
            return 0;
        return 1;
    }

    int checkValid(State *ps, State *s)
    {
        double timeinterval = 20;
        int cx = ps->x, cy = ps->y;
        double dx = (s->x - cx) / timeinterval, dy = (s->y - cy) / timeinterval;
        for (int i = 0; i <= timeinterval; i++)
        {
            double x = cx + dx * i, y = cy + dy * i;

            if (x < 0 || x >= boardw || y < 0 || y >= boardh)
                return 0;
            dummy_static_obs.set(int(x), int(y));
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
        htable_base = new float[boardsize];
        dtablebase = new double[boardsize];
        derrtablebase = new double[boardsize];
        bool checkTable[boardsize];
        for (int i = 0; i < boardsize; i++)
        {
            htable_base[i] = FLT_MAX;
            checkTable[i] = false;
        }

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



    void print_table_helper(int x, int y, float* table)
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
                else if (debug.find(point(j, i,0,0)) != debug.end())
                    cerr << "\033[0;32m";
                else if (debug1.find(point(j, i,0,0)) != debug.end())
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
        print_table_helper(x,y,htable_base);
    }

    // void printHStable(int x, int y)
    // {
    //     print_table_helper(x,y,htable_static);
    // }

    // void printDtable(int x, int y)
    // {
    //     print_table_helper(x,y,dtable);
    // }

    // void printDerrtable(int x, int y)
    // {
    //     print_table_helper(x,y,derrtable);
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