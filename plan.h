#ifndef __PLAN_H__
#define __PLAN_H__

#include <iostream>
#include <unordered_set>
#include <vector>
#include "state.h"
#include <queue>
#include <cfloat>

using namespace std;

class Plan
{
  protected:
    int LOOKAHEAD = 10, Collision_cost = 200, decay = 20;
    int boardw, boardh, startTime;
    double sqrt2 = 1.41421356237;
    float *htable;
    State goal_plan;
    State start_plan;

    vector<DynamicObstacle> dynamicObstacles;
    unordered_set<StaticObstacle> staticObstacles;
    StaticObstacle dummy_static_obs;
    vector<State> path;

    //pltastar
    float *htable_static;
    //fhat
    double * dtable, *derrtable;
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
            return s.fakeh < s1.fakeh;
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
        return getDistance(s->x,s->y);
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

    int cost_d(State *s)
    {
        for (DynamicObstacle dynamic : dynamicObstacles)
        {
            int x = dynamic.x, y = dynamic.y, left = dynamic.left, right = dynamic.right, t = s->time - startTime;
            int diff = right - left;
            if (dynamic.angle == 0)
            {
                if (t < right - x)
                {
                    x += t;
                }
                else
                {
                    t -= (right - x);

                    int remain = t % diff;
                    int iteration = t / diff;
                    if (iteration % 2 == 0)
                        x = right - remain;

                    else
                        x = left + remain;

                    //start from left
                }
            }
            else
            {
                if (x - left > t)
                {
                    x -= t;
                }
                else
                {
                    t -= (x - left);
                    int remain = t % diff;
                    int iteration = t / diff;
                    if (iteration % 2 == 0)
                        x = left + remain;
                    else
                        x = right - remain;

                    //start from right
                }
            }

            if (x == s->x && y == s->y)
            {
                return Collision_cost;
            }
        }
        return 0;
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
        htable = new float[boardsize];
        dtable = new double[boardsize];
        derrtable = new double[boardsize];
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
                    dtable[index] = derrtable[index] = getDistance(j, i);
                }
            }
        }

        htable_static = new float[boardw * boardh];
        memcpy(htable_static, htable, sizeof(float) * boardw * boardh);
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