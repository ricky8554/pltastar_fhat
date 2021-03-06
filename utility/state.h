#ifndef __STATE_H__
#define __STATE_H__

#include <cmath>
#include <iostream>
#include <unordered_set>

struct point_t
{
    int x;
    int y;
    int time;

    point_t(int x1, int y1, int t1)
    {
        x = x1;
        y = y1;
        time = t1;
    };
    point_t(){};

    bool operator==(const point_t &s) const
    {
        if (x == s.x && y == s.y && time == s.time)
            return true;
        return false;
    }

    void set(int x1, int y1)
    {
        x = x1;
        y = y1;
    }
};

namespace std
{
template <>
struct hash<point_t>
{
    hash()
    {
    }
    size_t operator()(const point_t &c) const
    {
        static hash<int> hashf;
        size_t ret = 0;
        ret ^= hashf(c.x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(c.y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(c.time) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    }
};
} // namespace std

class State
{
  public:
    int x, y, qindex, depth;
    int qtime;
    mutable int time;
    double fakeh, fakec, tempc;
    std::unordered_set<point_t> pred;

    State(double x, double y)
        : x(x), y(y), time(0){};
    State(double x, double y, int time)
        : x(x), y(y), time(time){};
    State(double x, double y, int time, std::unordered_set<point_t> pred)
        : x(x), y(y), time(time), pred(pred){};

    State()
        : x(-1), y(-1), time(0){};

    virtual ~State(){};

    bool operator==(const State &s) const
    {
        if (x == s.x && y == s.y && this->time == s.time)
            return true;
        return false;
    }

    virtual const double f() const
    {
        return fakeh + fakec;
    };
};

struct Dynamicxy
{
    double x, y, radius;
    Dynamicxy(double x, double y, double radius)
        : x(x), y(y), radius(radius){};
    Dynamicxy(){};

    double operator-(const Dynamicxy &d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }
};

struct movement
{
    
    double heading, speed;
    int steps;
    movement()
        : heading(0),speed(0),steps(0){};

    movement(double heading, double speed, int step)
        : heading(heading),speed(speed),steps(step){};
};

struct DynamicObstacle
{
    std::vector<movement> instructions;
    double x, y, estimate_h, estimate_s;
    double heading, speed;
    int remain;
    int current;
    int total;
    DynamicObstacle(int x, int y, double heading, double speed)
        : x(x), y(y), estimate_h(0), estimate_s(0), heading(heading),speed(speed){};

    DynamicObstacle(int x, int y, int total)
        : x(x), y(y), estimate_h(0), estimate_s(0), heading(0),speed(0), current(0), total(total){};

    DynamicObstacle()
        : estimate_h(0),estimate_s(0){};

    Dynamicxy getxy()
    {
        return (Dynamicxy(x, y, 1));
    };

    double operator-(const DynamicObstacle d)
    {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }

    friend double operator-(Dynamicxy d, const DynamicObstacle &d1)
    {
        return sqrt((d.x - d1.x) * (d.x - d1.x) + (d.y - d1.y) * (d.y - d1.y));
    }

    friend double operator-(DynamicObstacle d, const Dynamicxy &d1)
    {
        return sqrt((d.x - d1.x) * (d.x - d1.x) + (d.y - d1.y) * (d.y - d1.y));
    }
};

struct StaticObstacle
{
    int x;
    int y;

    StaticObstacle(int x1, int y1)
    {
        x = x1;
        y = y1;
    };
    StaticObstacle(){};

    bool operator==(const StaticObstacle &s) const
    {
        if (x == s.x && y == s.y)
            return true;
        return false;
    }

    void set(int x1, int y1)
    {
        x = x1;
        y = y1;
    }
};

struct point
{
    double x;
    double y;

    point(double x1, double y1)
    {
        x = x1;
        y = y1;
    };
    point(){};

    bool operator==(const point &s) const
    {
        if (x == s.x && y == s.y)
            return true;
        return false;
    }

    void set(double x1, double y1)
    {
        x = x1;
        y = y1;
    }
};

namespace std
{

template <>
struct hash<State>
{
    hash()
    {
    }
    size_t operator()(const State &c) const
    {
        static hash<int> hashf;
        size_t ret = 0;
        ret ^= hashf(c.x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(c.y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(c.time) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    }
};
template <>
struct hash<DynamicObstacle>
{
    hash()
    {
    }
    size_t operator()(const DynamicObstacle &c) const
    {
        static hash<int> hashf;
        size_t ret = 0;
        ret ^= hashf(c.x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(c.y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    }
};

template <>
struct hash<StaticObstacle>
{
    hash()
    {
    }
    size_t operator()(const StaticObstacle &c) const
    {
        static hash<int> hashf;
        size_t ret = 0;
        ret ^= hashf(c.x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(c.y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    }
};
template <>
struct hash<point>
{
    hash()
    {
    }
    size_t operator()(const point &c) const
    {
        static hash<int> hashf;
        size_t ret = 0;
        ret ^= hashf(c.x) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        ret ^= hashf(c.y) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
        return ret;
    }
};

} // namespace std

#endif