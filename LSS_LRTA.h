#ifndef __LSS_H__
#define __LSS_H__

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

class State_LSS : public State
{
  public:
    mutable double cost, h;
    State_LSS *parent;

    State_LSS(double x, double y)
        : State(x, y), cost(0), h(0){};
    State_LSS(double x, double y, int time)
        : State(x, y, time), cost(0), h(0){};
    State_LSS(int x, int y, double cost, double h)
        : State(x, y), cost(cost), h(h){};
    State_LSS(int x, int y, double cost, double h, int time)
        : State(x, y, time), cost(cost), h(h){};
    State_LSS(State &s)
        : State(s){};
    State_LSS(State_LSS *s)
        : State(s->x, s->y, s->time), cost(s->cost), h(s->h){};
    State_LSS(){};

    ~State_LSS(){};

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

    bool operator==(const State_LSS &s) const
    {
        if (x == s.x && y == s.y && this->time == s.time)
            return true;
        return false;
    }
    const double f() const override
    {
        return cost + h;
    };

    bool operator<(const State_LSS &s) const
    {
        return (cost + h < s.cost + s.h);
    }

    bool operator>(const State_LSS &s) const
    {
        return (cost + h > s.cost + s.h);
    }

    double operator-(const State_LSS &d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }
};

class Lss_Lrta : public Plan
{

  public:
    // Default constructor

    Lss_Lrta()
    {
        ios_base::sync_with_stdio(false);
    };

    ~Lss_Lrta()
    {
        auto it = expandState.begin();
        while (it != expandState.end())
        {
            State_LSS *elem = it->key;
            ++it;
            expandState.erase(elem);
            delete elem;
        }
        delete dummy;
        delete[] htable;
        delete[] derrtable;
        delete[] dtable;
    };

    void setStartGoal(State starta, State goala, int bordw, int bordh) override;

    int plan(State start) override; //from lss lrta *

    void setStatic(unordered_set<StaticObstacle> &s) override;

    void setDynamic(vector<DynamicObstacle> &d) override;


    unordered_set<State> getSTATE() override
    {
        unordered_set<State> ret;
        for (auto i : expandState)
        {
            State s = State(i.key->x,i.key->y,i.key->time);
            s.fakec = i.key->cost;
            s.fakeh = i.key->h;
            ret.insert(s);
        }
        return ret;
    }

    int ASTAR(State s);

    int update_h();

    State_LSS *pickBest();

    mutex mtx;

  private:
    State_LSS *start, *goal, *dummy;
    PQueue<State_LSS *> open;
    PQueue<State_LSS *> goalQ;
    HashSet<State_LSS *> opencheck = HashSet<State_LSS *>(&Hash, &equal, 0);
    HashSet<State_LSS *> close = HashSet<State_LSS *>(&Hash, &equal, 0);
    HashSet<State_LSS *> expandState = HashSet<State_LSS *>(&Hash, &equal, 0);
    static hash<int> hashf;

    static size_t Hash(State_LSS *const &s)
    {
        size_t ret = 0;
        ret^= hashf(s->x) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        ret^= hashf(s->y) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        ret^= hashf(s->time) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        return ret;
    };
    static unsigned int compare1(State_LSS *&s1, State_LSS *&s2)
    {
        return s1->h < s2->h; //chage to h_hat?
    };

    static unsigned int compare(State_LSS *&s1, State_LSS *&s2)
    {
        return s1->f() < s2->f();
    };

    static unsigned int compare(State_LSS &s1, State_LSS &s2)
    {
        return s1.f() < s2.f();
    };

    static unsigned int compare1(State_LSS &s1, State_LSS &s2)
    {
        return s1.h < s2.h; //chage to h_hat?
    };

    static unsigned int equal(State_LSS *const &s1, State_LSS *const &s2)
    {
        return *s1 == *s2;
    };
};

#endif