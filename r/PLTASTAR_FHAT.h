#ifndef __PLRFHAT_H__
#define __PLRFHAT_H__

#include <stdio.h>
#include <iostream>
#include "PQueue.h"
#include "HashSet.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <float.h>
#include "state.h"
#include "plan.h"

using namespace std;

class State_PLRTA_FHAT : public State
{
  public:
    mutable double cost_s, cost_d, h_s, h_d, derr, h_error, d;
    State_PLRTA_FHAT *parent;

    State_PLRTA_FHAT(double x, double y)
        : State(x, y), cost_s(0), cost_d(0), h_s(0), h_d(0), derr(0), h_error(0), d(0){};
    State_PLRTA_FHAT(double x, double y, int time)
        : State(x, y, time), cost_s(0), cost_d(0), h_s(0), h_d(0), derr(0), h_error(0), d(0){};

    State_PLRTA_FHAT(int x, int y, double cost_s, double cost_d, double h_s, double h_d, int time)
        : State(x, y, time), cost_s(cost_s), cost_d(cost_d), h_s(h_s), h_d(h_d), derr(0), h_error(0), d(0){};

    State_PLRTA_FHAT(int x, int y, double cost_s, double cost_d, double h_s, double h_d, int time, State_PLRTA_FHAT *parent, unordered_set<point_t> pred)
        : State(x, y, time,pred), cost_s(cost_s), cost_d(cost_d), h_s(h_s), h_d(h_d), derr(0), h_error(0), d(0), parent(parent){};

    State_PLRTA_FHAT(State &s)
        : State(s){};

    State_PLRTA_FHAT(State_PLRTA_FHAT *s)
        : State(s->x, s->y, s->time,s->pred), cost_s(s->cost_s), cost_d(s->cost_d), h_s(s->h_s), h_d(s->h_d), derr(s->derr), h_error(s->h_error), d(s->d), parent(s->parent){};

    State_PLRTA_FHAT(){};

    ~State_PLRTA_FHAT(){};

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

    bool operator==(const State_PLRTA_FHAT &s) const
    {
        if (x == s.x && y == s.y && this->time == s.time)
            return true;
        return false;
    }

    const double f() const override
    {
        return cost_s + cost_d + h_s + h_d;
    }

    const double fhat() const
    {
        return cost_s + cost_d + h_s + h_d + h_error;
    };

    bool operator<(const State_PLRTA_FHAT &s) const
    {
        return (cost_s + cost_d + h_s + h_d + h_error < s.cost_s + s.cost_d + s.h_s + s.h_d + s.h_error);
    }

    bool operator>(const State_PLRTA_FHAT &s) const
    {
        return (cost_s + cost_d + h_s + h_d + h_error > s.cost_s + s.cost_d + s.h_s + s.h_d + s.h_error);
    }

    double operator-(const State_PLRTA_FHAT &d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }
};

struct State_PLRTA_FHAT_Static : public State_PLRTA_FHAT
{
    State_PLRTA_FHAT_Static(State_PLRTA_FHAT *s)
        : State_PLRTA_FHAT(s){};
    State_PLRTA_FHAT_Static(State_PLRTA_FHAT_Static *s)
        : State_PLRTA_FHAT(s->x, s->y, s->cost_s, s->cost_d, s->h_s, s->h_d, s->time, s->parent, s->pred){};
    State_PLRTA_FHAT_Static(){};
    unordered_set<point> pred_static;

    void set(State_PLRTA_FHAT *s)
    {
        x = s->x;
        y = s->y;
    };



    void set(point s)
    {
        x = s.x;
        y = s.y;
    };

    bool operator==(const State_PLRTA_FHAT_Static &s) const
    {
        if (x == s.x && y == s.y)
            return true;
        return false;
    }

    const double f() const override
    {
        return cost_s + cost_d + h_s + h_d;
    }

    const double fhat() const
    {
        return cost_s + cost_d + h_s + h_d + h_error;
    };

    bool operator<(const State_PLRTA_FHAT_Static &s) const
    {
        return (cost_s + cost_d + h_s + h_d + h_error < s.cost_s + s.cost_d + s.h_s + s.h_d + s.h_error);
    }

    bool operator>(const State_PLRTA_FHAT_Static &s) const
    {
        return (cost_s + cost_d + h_s + h_d + h_error > s.cost_s + s.cost_d + s.h_s + s.h_d + s.h_error);
    }

    double operator-(const State_PLRTA_FHAT_Static &d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }
};

class PLTASTAR_FHAT : public Plan
{

  public:
    // Default constructor
    PLTASTAR_FHAT()
    {
        ios_base::sync_with_stdio(false);
    };

    ~PLTASTAR_FHAT()
    {
        auto it = expandState.begin();
        while (it != expandState.end())
        {
            State_PLRTA_FHAT *elem = it->key;
            ++it;
            expandState.erase(elem);
            delete elem;
        }
        delete dummy;
        delete dummy_static;
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
            State s = State(i.key->x, i.key->y, i.key->time);
            s.fakec = i.key->cost_s + i.key->cost_d;
            s.fakeh = i.key->h_s + i.key->h_d;
            ret.insert(s);
        }
        return ret;
    }

    int ASTAR(State s);

    int update_h_static(PQueue<State_PLRTA_FHAT *> open, HashSet<State_PLRTA_FHAT *> opencheck, HashSet<State_PLRTA_FHAT *> close);

    int update_h_dynamic();

    double getDerr(State_PLRTA_FHAT *s);

    double getD(State_PLRTA_FHAT *s);

    State_PLRTA_FHAT *pickBest();

    mutex mtx;

  private:
    double herr = 0, derr = 0;
    State_PLRTA_FHAT *start, *goal, *dummy;
    State_PLRTA_FHAT_Static *dummy_static;
    PQueue<State_PLRTA_FHAT *> open;
    PQueue<State_PLRTA_FHAT *> goalQ;
    HashSet<State_PLRTA_FHAT *> opencheck = HashSet<State_PLRTA_FHAT *>(&Hash, &equal, 0);
    HashSet<State_PLRTA_FHAT *> close = HashSet<State_PLRTA_FHAT *>(&Hash, &equal, 0);
    HashSet<State_PLRTA_FHAT *> expandState = HashSet<State_PLRTA_FHAT *>(&Hash, &equal, 0);
    static hash<int> hashf;

    static size_t Hash(State_PLRTA_FHAT *const &s)
    {
        size_t ret = 0;
        ret ^= hashf(s->x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(s->y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(s->time) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    };

    static size_t Hash_static(State_PLRTA_FHAT_Static *const &s)
    {
        size_t ret = 0;
        ret ^= hashf(s->x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(s->y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    };

    static unsigned int compare(State_PLRTA_FHAT *&s1, State_PLRTA_FHAT *&s2)
    {
        return s1->fhat() < s2->fhat();
    };

    static unsigned int compare(State_PLRTA_FHAT &s1, State_PLRTA_FHAT &s2)
    {
        return s1.fhat() < s2.fhat();
    };

    static unsigned int compare1(State_PLRTA_FHAT *&s1, State_PLRTA_FHAT *&s2)
    {
        return s1->h_s + s1->h_d + s1->h_error < s2->h_s + s2->h_d + s2->h_error; //chage to h_hat?
    };

    static unsigned int compare1(State_PLRTA_FHAT &s1, State_PLRTA_FHAT &s2)
    {
        return s1.h_s + s1.h_d + s1.h_error < s2.h_s + s2.h_d + s2.h_error; //chage to h_hat?
    };

    static unsigned int compare2(State_PLRTA_FHAT *&s1, State_PLRTA_FHAT *&s2)
    {
        return s1->h_d < s2->h_d; //chage to h_hat?
    };

    static unsigned int compare2(State_PLRTA_FHAT &s1, State_PLRTA_FHAT &s2)
    {
        return s1.h_d < s2.h_d; //chage to h_hat?
    };

    static unsigned int compare_static(State_PLRTA_FHAT *&s1, State_PLRTA_FHAT *&s2)
    {
        return s1->h_s < s2->h_s; //chage to h_hat?
        // return s1->h_s + s1->h_error < s2->h_s+ s2->h_error;
    };

    static unsigned int compare_static(State_PLRTA_FHAT &s1, State_PLRTA_FHAT &s2)
    {
        return s1.h_s < s2.h_s; //chage to h_hat?
        // return s1.h_s + s1.h_error < s2.h_s + s2.h_error;
    };

    static unsigned int compare_static(State_PLRTA_FHAT_Static &s1, State_PLRTA_FHAT_Static &s2)
    {
        return s1.h_s < s2.h_s;
        // return s1.h_s + s1.h_error < s2.h_s + s2.h_error;
    };

    static unsigned int compare_static(State_PLRTA_FHAT_Static *&s1, State_PLRTA_FHAT_Static *&s2)
    {
        return s1->h_s < s2->h_s;
        // return s1->h_s + s1->h_error < s2->h_s + s2->h_error;
    };

    static unsigned int equal(State_PLRTA_FHAT *const &s1, State_PLRTA_FHAT *const &s2)
    {
        return *s1 == *s2;
    };

    static unsigned int equal_static(State_PLRTA_FHAT_Static *const &s1, State_PLRTA_FHAT_Static *const &s2)
    {
        return *s1 == *s2;
    };
};

#endif