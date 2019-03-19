#ifndef __PLR_H__
#define __PLR_H__

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

class State_PLRTA : public State
{
  public:
    mutable double cost_s, cost_d, h_s, h_d;
    State_PLRTA *parent;

    State_PLRTA(int x, int y, int dx, int dy)
        : State(x, y, dx, dy), cost_s(0), cost_d(0), h_s(0), h_d(0){};
    State_PLRTA(int x, int y, int dx, int dy, int time)
        : State(x, y, dx, dy, time), cost_s(0), cost_d(0), h_s(0), h_d(0){};

    State_PLRTA(int x, int y, int dx, int dy, double cost_s, double cost_d, double h_s, double h_d, int time)
        : State(x, y, dx, dy, time), cost_s(cost_s), cost_d(cost_d), h_s(h_s), h_d(h_d){};

    State_PLRTA(int x, int y, int dx, int dy, double cost_s, double cost_d, double h_s, double h_d, int time, State_PLRTA *parent, unordered_set<point_t> pred)
        : State(x, y, dx, dy, time,pred), cost_s(cost_s), cost_d(cost_d), h_s(h_s), h_d(h_d), parent(parent){};

    State_PLRTA(State &s)
        : State(s){};

    State_PLRTA(State_PLRTA *s)
        : State(s->x, s->y, s->dx, s->dy, s->time,s->pred), cost_s(s->cost_s), cost_d(s->cost_d), h_s(s->h_s), h_d(s->h_d), parent(s->parent){};

    State_PLRTA(){};

    ~State_PLRTA(){};


    bool operator==(const State_PLRTA &s) const
    {
        return (x == s.x && y == s.y && dx == s.dx && dy == s.dy && this->time == s.time);
    }

    const double f() const override
    {
        return cost_s + cost_d + h_s + h_d;
    }

    bool operator<(const State_PLRTA &s) const
    {
        return (f() < s.f());
    }

    bool operator>(const State_PLRTA &s) const
    {
        return (f() > s.f());
    }

    double operator-(const State_PLRTA &d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }

    friend ostream &operator<<(ostream &os, const State_PLRTA &state)
    {
        os << "X:" << setw(3) << left << state.x
           << "Y:" << setw(3) << left << state.y
           << "x:" << setw(3) << left << state.dx
           << "y:" << setw(3) << left << state.dy
           << "G:" << setw(3) << left << state.cost_s
           << "H:" << setw(3) << left << state.h_s
           << "g:" << setw(3) << left << state.cost_d
           << "h:" << setw(3) << left << state.h_d
           << "F:" << setw(3) << left << state.f()
           << "T:" << setw(4) << left << state.time
           << "D:" << state.depth;
        return os;
    }
};

struct State_PLRTA_Static : public State_PLRTA
{
    State_PLRTA_Static(State_PLRTA *s)
        : State_PLRTA(s){};
    State_PLRTA_Static(State_PLRTA_Static *s)
        : State_PLRTA(s->x, s->y,s->dx,s->dy, s->cost_s, s->cost_d, s->h_s, s->h_d, s->time, s->parent,s->pred){};
    State_PLRTA_Static(){};
    unordered_set<point> pred_static;

    void set(State_PLRTA *s)
    {
        x = s->x;
        y = s->y;
        dx = s->dx;
        dy = s->dy;
    };

    void set(point s)
    {
        x = s.x;
        y = s.y;
        dx = s.dx;
        dy = s.dy;
    };

    bool operator==(const State_PLRTA_Static &s) const
    {
        return (x == s.x && y == s.y && dx == s.dx && dy == s.dy);
    }

    const double f() const
    {
        return cost_s + cost_d + h_s + h_d;
    }

    bool operator<(const State_PLRTA_Static &s) const
    {
        return (f() < s.f());
    }

    bool operator>(const State_PLRTA_Static &s) const
    {
        return (f() > s.f());
    }

    double operator-(const State_PLRTA_Static &d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }
};

class PLTASTAR : public Plan
{

  public:
    // Default constructor
    PLTASTAR()
    {
        ios_base::sync_with_stdio(false);
    };

    ~PLTASTAR()
    {
        auto it = expandState.begin();
        while (it != expandState.end())
        {
            State_PLRTA *elem = it->key;
            ++it;
            expandState.erase(elem);
            delete elem;
        }
        delete dummy;
        delete dummy_static;
        // delete[] htable;
        // delete[] derrtable;
        // delete[] dtable;
    };

    void setStartGoal(State starta, State goala, int bordw, int bordh) override;

    int plan(State start) override; //from lss lrta *

    void setStatic(unordered_set<StaticObstacle> &s) override;

    // void setDynamic(vector<DynamicObstacle> &d) override;

    unordered_set<State> getSTATE() override
    {
        unordered_set<State> ret;
        for (auto i : expandState)
        {
            State s = State(i.key->x, i.key->y, i.key->dx, i.key->dy, i.key->time);
            s.fakec = i.key->cost_s + i.key->cost_d;
            s.fakeh = i.key->h_s + i.key->h_d;
            ret.insert(s);
        }
        return ret;
    }


    int ASTAR(State s);

    int update_h_static(PQueue<State_PLRTA*> open, HashSet<State_PLRTA*> opencheck, HashSet<State_PLRTA*> close);

    int update_h_dynamic();

    State_PLRTA *pickBest();

    mutex mtx;

  private:
    State_PLRTA *start, *goal, *dummy;
    State_PLRTA_Static* dummy_static;
    PQueue<State_PLRTA *> open;
    PQueue<State_PLRTA *> goalQ;
    HashSet<State_PLRTA *> opencheck = HashSet<State_PLRTA *>(&Hash, &equal, 0);
    HashSet<State_PLRTA *> close = HashSet<State_PLRTA *>(&Hash, &equal, 0);
    HashSet<State_PLRTA *> expandState = HashSet<State_PLRTA *>(&Hash, &equal, 0);
    static hash<int> hashf;
    
    static size_t Hash(State_PLRTA *const &s)
    {
        size_t ret = 0;
        ret^= hashf(s->x) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        ret^= hashf(s->y) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        ret ^= hashf(s->dx) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(s->dy) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret^= hashf(s->time) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        return ret;
    };

    static size_t Hash_static(State_PLRTA_Static *const &s)
    {
        size_t ret = 0;
        ret^= hashf(s->x) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        ret^= hashf(s->y) + 0x9e3779b9 + (ret<< 6) + (ret>> 2);
        ret ^= hashf(s->dx) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(s->dy) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    };

    

    static unsigned int compare(State_PLRTA *&s1, State_PLRTA *&s2)
    {
        return s1->f() < s2->f();
    };

    static unsigned int compare(State_PLRTA &s1, State_PLRTA &s2)
    {
        return s1.f() < s2.f();
    };

    static unsigned int compare1(State_PLRTA *&s1, State_PLRTA *&s2)
    {
        return s1->h_s + s1->h_d < s2->h_s+ s2->h_d; //chage to h_hat?
    };

    static unsigned int compare1(State_PLRTA &s1, State_PLRTA &s2)
    {
        return s1.h_s + s1.h_d < s2.h_s + s2.h_d  ; //chage to h_hat?
    };

    static unsigned int compare2(State_PLRTA *&s1, State_PLRTA *&s2)
    {
        return s1->h_d < s2->h_d; //chage to h_hat?
    };

    static unsigned int compare2(State_PLRTA &s1, State_PLRTA &s2)
    {
        return s1.h_d < s2.h_d; //chage to h_hat?
    };

    static unsigned int compare_static(State_PLRTA *&s1, State_PLRTA *&s2)
    {
        return s1->h_s < s2->h_s; //chage to h_hat?
    };

    static unsigned int compare_static(State_PLRTA &s1, State_PLRTA &s2)
    {
        return s1.h_s < s2.h_s; //chage to h_hat?
    };


    static unsigned int compare_static(State_PLRTA_Static &s1, State_PLRTA_Static &s2)
    {
        return s1.h_s < s2.h_s;
    };

    static unsigned int compare_static(State_PLRTA_Static *&s1, State_PLRTA_Static *&s2)
    {
        return s1->h_s < s2->h_s;
    };

    static unsigned int equal(State_PLRTA *const &s1, State_PLRTA *const &s2)
    {
        return *s1 == *s2;
    };

    static unsigned int equal_static(State_PLRTA_Static *const &s1, State_PLRTA_Static *const &s2)
    {
        return *s1 == *s2;
    };
};

#endif