#ifndef __base_H__
#define __base_H__

#include <stdio.h>
#include <iostream>
#include "PQueue.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <float.h>
#include "state.h"
#include "plan.h"
#include "HashSet.h"

using namespace std;

class State_Base : public State
{
  public:
    mutable double cost, h;
    State_Base *parent;

    State_Base(double x, double y)
        : State(x, y), cost(0), h(0){};
    State_Base(double x, double y, int time)
        : State(x, y, time), cost(0), h(0){};
    State_Base(int x, int y, double cost, double h)
        : State(x, y), cost(cost), h(h){};
    State_Base(int x, int y, double cost, double h, int time)
        : State(x, y, time), cost(cost), h(h){};
    State_Base(State &s)
        : State(s){};
    State_Base(State_Base *s)
        : State(s->x, s->y, s->time), cost(s->cost), h(s->h){};
    State_Base(){};

    ~State_Base(){};

    void set(State &s)
    {
        x = s.x;
        y = s.y;
        time = s.time;
    };

    void set(point_t &s)
    {
        x = s.x;
        y = s.y;
        time = s.time;
    };

    void set(double x1, double y1, int time1)
    {
        x = x1;
        y = y1;
        time = time1;
    };

    bool operator==(const State_Base &s) const
    {
        if (x == s.x && y == s.y && this->time == s.time)
            return true;
        return false;
    }
    const double f() const override
    {
        return cost + h;
    };

    bool operator<(const State_Base &s) const
    {
        return (cost + h < s.cost + s.h);
    }

    bool operator>(const State_Base &s) const
    {
        return (cost + h > s.cost + s.h);
    }

    double operator-(const State_Base &d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }

    friend ostream &operator<<(ostream &os, const State_Base &state)
    {
        os << "X:" << setw(3) << left << state.x
           << "Y:" << setw(3) << left << state.y
           << "G:" << setw(3) << left << state.cost
           << "H:" << setw(3) << left << state.h
           << "F:" << setw(3) << left << state.f()
           << "T:" << setw(4) << left << state.time
           << "D:" << state.depth;
        return os;
    }
};

class Base_plan : public Plan
{

  public:
    // Default constructor

    Base_plan()
    {
        ios_base::sync_with_stdio(false);
    };

    ~Base_plan()
    {
        // auto it = expandState.begin();
        // while (it != expandState.end())
        // {
        //     State_Base *elem = it->key;
        //     ++it;
        //     expandState.erase(elem);
        //     delete elem;
        // }
        delete dummy;
        delete[] htable;
        delete[] derrtable;
        delete[] dtable;
    };

    void setStartGoal(State s, State g, int bordw, int bordh) override
    {
        boardw = bordw;
        boardh = bordh;

        goalQ.setUpCompare(&compare1);

        start = new State_Base(s);
        goal = new State_Base(g);
        dummy = new State_Base();

        goal->time = INT_MAX;
        start->time = 0;
        goal->cost = DBL_MAX;
        start->cost = 0;

        constructHtable(State(start->x, goal->y), State(goal->x, goal->y));

        start->h = h_value(start);
        goal->h = 0;
    };

    int plan(State requestStart) override
    {
        startTime = requestStart.time;
        State_Base *s, *sgoal;

        s = sgoal = ASTAR();

        path = vector<State>();
        while (s != start)
        {
            path.push_back(*s);
            s = s->parent;
        }

        start = sgoal;
        return !(start == goal);
    }

    void setStatic(unordered_set<StaticObstacle> &s) override
    {
        staticObstacles = s;
    };

    void update_dynamic(DynamicObstacle &dynamic)
    {
        double speed, heading;
        dynamic.estimate_h = heading = dynamic.heading;
        dynamic.estimate_s = speed = dynamic.speed;
        dynamic.x += cos(heading) * speed;
        dynamic.y += sin(heading) * speed;
        if (--dynamic.remain == 0)
        {
            dynamic.current += 1;
            movement m = dynamic.instructions[dynamic.current % dynamic.total];
            dynamic.heading = m.heading;
            dynamic.speed = m.speed;
            dynamic.remain = m.steps;
        }
    }

    int cost_dynamic(State_Base *ps, State_Base *s)
    {
        vector<DynamicObstacle> v, pv;
        if (dobs_map.find(s->time) == dobs_map.end())
        {
            if(!ps->time)
                dobs_map[ps->time] = dynamicObstacles;
            
            v = pv = dobs_map[ps->time];
            for (DynamicObstacle &d : v)
                update_dynamic(d);
            dobs_map[s->time] = v;
        }
        else
        {
            v = dobs_map[s->time];
            pv = dobs_map[ps->time];
        }

        int factor1;
        double timeinterval = factor1 = 10;
        int cx = ps->x, cy = ps->y, fx = s->x, fy = s->y;
        double diffx = (fx - cx) / timeinterval, diffy = (fy - cy) / timeinterval;
        for (int k = 0; k < v.size(); k++)
        {
            double ddiffx = (v[k].x - pv[k].x) / timeinterval, ddiffy = (v[k].y - pv[k].y) / timeinterval;
            double dcx = pv[k].x, dcy = pv[k].y;
            for (int i = 1; i <= factor1; i++)
            {
                double x = cx + diffx * i, y = cy + diffy * i;
                double dx = dcx + ddiffx * i, dy = dcy + ddiffy * i;
                double dix = dx - x;
                double diy = dy - y;
                double dist = dix * dix + diy * diy;
                if(dist < 2.25)
                {
                    return Collision_cost;
                }
            }
        }

        return 0;
    }

    // void setDynamic(vector<DynamicObstacle> &d) override;

    unordered_set<State> getSTATE() override
    {
        unordered_set<State> ret;
        // for (auto i : expandState)
        // {
        //     State s = State(i.key->x, i.key->y, i.key->time);
        //     s.fakec = i.key->cost;
        //     s.fakeh = i.key->h;
        //     ret.insert(s);
        // }
        return ret;
    }

    State_Base *ASTAR()
    {
        State_Base *state;
        open.clear();
        open.setUpCompare(&compare);
        opencheck.clear();
        close.clear();

        start->cost = 0;
        start->depth = 0;
        open.push(start);
        opencheck.insert(start);

        do
        {
            // cerr << "IN" << endl;
            state = open.pop();
            // cerr << "State " << *state << endl;
            // cerr << "time " << state->time << " size " << open.size() << " look " << LOOKAHEAD << endl;
            if (state->time == LOOKAHEAD)
            {
                return state;
            }
            opencheck.erase(state);
            close.insert(state);
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {

                    dummy->set(state->x + i, state->y + j, state->time + 1);
                    if (checkValid(dummy) && !close.find(dummy))
                    {

                        State_Base *child_state = (opencheck.find(dummy)) ? opencheck[dummy] : new State_Base(state->x + i, state->y + j, DBL_MAX, h_value(dummy), state->time + 1);
                        double next_cost = state->cost + cost(state, child_state) + cost_dynamic(state, child_state);

                        if (child_state->cost > next_cost)
                        {
                            child_state->cost = next_cost;
                            child_state->parent = state;
                            child_state->depth = state->depth + 1;

                            if (!opencheck.find(child_state))
                            {
                                opencheck.insert(child_state);
                                open.push(child_state);
                            }
                            else
                            {
                                open.moveUP(child_state->qindex);
                            }
                        }
                    }
                }
            }
            // cerr << "OUT" << endl;
        } while (true);
    }

  private:
    State_Base *start, *goal, *dummy;
    unordered_map<int, vector<DynamicObstacle>> dobs_map;
    PQueue<State_Base *> open;
    PQueue<State_Base *> goalQ;
    HashSet<State_Base *> opencheck = HashSet<State_Base *>(&Hash, &equal, 0);
    HashSet<State_Base *> close = HashSet<State_Base *>(&Hash, &equal, 0);
    // HashSet<State_Base *> expandState = HashSet<State_Base *>(&Hash, &equal, 0);
    static hash<int> hashf;

    static size_t Hash(State_Base *const &s)
    {
        size_t ret = 0;
        ret ^= hashf(s->x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(s->y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(s->time) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    };
    static unsigned int compare1(State_Base *&s1, State_Base *&s2)
    {
        return s1->h < s2->h; //chage to h_hat?
    };

    static unsigned int compare(State_Base *&s1, State_Base *&s2)
    {
        return s1->f() < s2->f();
    };

    static unsigned int compare(State_Base &s1, State_Base &s2)
    {
        return s1.f() < s2.f();
    };

    static unsigned int compare1(State_Base &s1, State_Base &s2)
    {
        return s1.h < s2.h; //chage to h_hat?
    };

    static unsigned int equal(State_Base *const &s1, State_Base *const &s2)
    {
        return *s1 == *s2;
    };
};

#endif