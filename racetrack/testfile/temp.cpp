
#include "pltastar.h"
#include <iomanip>
#include <algorithm>
using namespace std;

#define LOOKAHEAD 10
#define Collision_cost 200
#define decay 20

double PLTASTAR::cost(State_PLRTA_Static &s, State_PLRTA_Static &s1)
{
    if (s.x == goal->x && s1.x == goal->x && s.y == goal->y && s1.y == goal->y)
        return 0;
    return 1;
}

double PLTASTAR::cost(State_PLRTA &s, State_PLRTA &s1)
{
    if (s.x == goal->x && s1.x == goal->x && s.y == goal->y && s1.y == goal->y)
        return 0;
    return 1;
}

int PLTASTAR::cost_d(State_PLRTA *s)
{
    for (DynamicObstacle dynamic : dynamicObstacles)
    {
        int x = dynamic.x, y = dynamic.y, t = s->time - startTime, direction = -1;
        if (dynamic.angle == 0)
            direction = 1;
        while (t--)
        {
            dummy_static_obs.set(x + direction, y);

            if (staticObstacles.find(dummy_static_obs) != staticObstacles.end())
            {
                if (direction == 1)
                    direction = -1;
                else
                    direction = 1;
                break;
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

        if (x == s->x && y == s->y)
        {
            return Collision_cost;
        }
    }
    return 0;
}

int PLTASTAR::checkValid(State_PLRTA *s)
{
    if (s->x < 0 || s->x >= boardw || s->y < 0 || s->y >= boardh)
        return 0;
    dummy_static_obs.set(s->x, s->y);
    if (staticObstacles.find(dummy_static_obs) != staticObstacles.end())
        return 0;

    return 1;
}
double PLTASTAR::h_value(State_PLRTA *s)
{
    return htable[getindex(s->x, s->y)];
}

double PLTASTAR::h_value_static(State_PLRTA *s)
{
    return htable_static[getindex(s->x, s->y)];
}

int PLTASTAR::ASTAR(State requestStart)
{
    State_PLRTA *state;
    dummy->set(requestStart);
    start = expandState[dummy];

    auto it = expandState.begin();

    while (it != expandState.end())
    {
        State_PLRTA *elem = it->key;
        ++it;
        if (elem->time < start->time && !(elem->x == goal->x && elem->y == goal->y))
        {
            expandState.erase(elem);
            delete elem;
        }
        else
        {
            elem->cost_s = DBL_MAX;
            elem->cost_d = DBL_MAX;
            elem->h_d = (elem->h_d > decay) ? elem->h_d - decay : 0;
        }
    }

    //clear open close & open check
    open.clear();
    open.setUpCompare(&compare);
    opencheck.clear();
    close.clear();

    start->cost_d = 0;
    start->cost_s = 0;
    start->h_s = h_value_static(start);

    int expansions = 0;
    //push start into open open check & expand state
    open.push(start);
    opencheck.insert(start);

    do
    {
        state = open.pop();
        opencheck.erase(state);
        close.insert(state);
        expansions += 1;
        //cout << "Pick: " << state.x << " " << state.y << " " << state.time << " " << state.h_d << " " << state.h_s << " "<< state.cost_s << " "<< state.cost_d<< endl;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                dummy->set(state->x + i, state->y + j, state->time + 1);
                if (checkValid(dummy) && !close.find(dummy))
                {
                    State_PLRTA *child_state = expandState[dummy];
                    if (!child_state)
                    {
                        child_state = new State_PLRTA(state->x + i, state->y + j, DBL_MAX, DBL_MAX, h_value_static(dummy), 0, state->time + 1);
                        expandState.insert(child_state);
                    }
                    child_state->h_s = h_value_static(child_state);
                    child_state->pred.insert(state);

                    double dcost = state->cost_d + cost_d(child_state);
                    double scost = state->cost_s + cost(state, child_state);
                    if (child_state->cost_s + child_state->cost_d > scost + dcost)
                    {

                        child_state->cost_s = scost;
                        child_state->cost_d = dcost;
                        child_state->parent = state;

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
    } while (expansions < LOOKAHEAD); //check collision for goal
    return 0;
}

int PLTASTAR::update_h_static(PQueue<State_PLRTA *> open1, HashSet<State_PLRTA *> opencheck1, HashSet<State_PLRTA *> close1)
{
    HashSet<State_PLRTA_Static *> close = HashSet<State_PLRTA_Static *>(&Hash_static, &equal_static, 0);
    HashSet<State_PLRTA_Static *> opencheck = HashSet<State_PLRTA_Static *>(&Hash_static, &equal_static, 0);
    PQueue<State_PLRTA_Static *> open;
    open.setUpCompare(&compare_static);

    for (auto it : close1)
    {
        dummy_static->set(it.key);
        State_PLRTA_Static *state = close[dummy_static];
        if (!state)
        {
            state = new State_PLRTA_Static(it.key);
            htable_static[getindex(state->x, state->y)] = DBL_MAX;

            close.insert(state);
        }
        else
        {
            for (auto i : it.key->pred)
            {
                state->pred.insert(i);
            }
        }
    }

    for (int i = 0; i < open1.size(); i++)
    {
        dummy_static->set(open1[i]);

        // if (close.find(dummy_static))
        // {
        //     //TODO:considered insert it into open !!!!
        //     if (htable_static[getindex(dummy_static->x, dummy_static->y)] > cost(dummy_static, dummy_static) + open1[i]->h_s)
        //         htable_static[getindex(dummy_static->x, dummy_static->y)] = cost(dummy_static, dummy_static) + open1[i]->h_s;
        // }

        // cerr << open1[i]->x << " " << open1[i]->y << endl;

        State_PLRTA_Static *state = close[dummy_static];

        if (!state)
        {
            state = new State_PLRTA_Static(open1[i]);
            opencheck.insert(state);
            open.push(state);
        }
        else
        {
            for (auto i : open1[i]->pred)
            {
                state->pred.insert(i);
            }
        }
    }
    while (close.size() != 0 && !open.empty())
    {

        State_PLRTA_Static *state = open.pop();
        opencheck.erase(state);
        close.erase(state);

        for (State_PLRTA *pred : state->pred)
        {
            dummy_static->set(pred);
            State_PLRTA_Static *pred_state = close[dummy_static];
            if (pred_state)
            {

                double pred_h = cost(pred_state, state) + state->h_s;
                // cerr << "PRED "<<pred_state->x << " " << pred_state->y<< " " << htable_static[getindex(pred_state->x, pred_state->y)]<<" " << state->h_s <<  endl;
                if (htable_static[getindex(pred_state->x, pred_state->y)] > pred_h)
                {
                    pred_state->h_s = pred_h;
                    htable_static[getindex(pred_state->x, pred_state->y)] = pred_h;
                    if (!opencheck.find(pred_state))
                    {
                        opencheck.insert(pred_state);
                        open.push(pred_state);
                    }
                    else
                    {
                        open.moveUP(pred_state->qindex);
                    }
                }
            }
        }

        delete state;
    }

    for (int i = 0; i < open.size(); i++)
        delete open[i];
    for (auto i : close)
        delete i.key;
    return 0;
}

int PLTASTAR::update_h_dynamic()
{
    for (auto it : close)
    {
        it.key->h_d = DBL_MAX;
    }

    open.setUpCompare(&compare2);

    while (close.size() != 0 && !open.empty())
    {
        State_PLRTA *state = open.pop();
        opencheck.erase(state);
        close.erase(state);

        for (State_PLRTA *pred : state->pred)
        {
            State_PLRTA *pred_state = close[pred];
            if (pred_state)
            {
                double dcost = cost_d(pred_state) + state->h_d;
                if (pred_state->h_d > dcost)
                {
                    pred_state->h_d = dcost;
                    if (!opencheck.find(pred_state))
                    {
                        opencheck.insert(pred_state);
                        open.push(pred_state);
                    }
                    else
                    {
                        open.moveUP(pred_state->qindex);
                    }
                }
            }
        }
    }
    return 0;
}

State_PLRTA *PLTASTAR::pickBest()
{
    double maxh = open.top()->f();
    while (!open.empty() && open.top()->f() <= maxh)
    {
        goalQ.push(open.pop());
    }

    State_PLRTA *s = goalQ.top();
    while (!goalQ.empty())
    {
        open.push(goalQ.pop());
    }

    return s;
}

int PLTASTAR::plan(State requestStart)
{
    startTime = requestStart.time;
    State_PLRTA *s, *sgoal;

    ASTAR(requestStart);

    if (open.empty())
        return 0;

    s = sgoal = pickBest();
    std::cout << "CHOSE: " << sgoal->x << "\t" << sgoal->y << "\t" << sgoal->cost_s << "\t" << sgoal->cost_d << "\t" << sgoal->h_s << "\t" << sgoal->h_d << "\t" << sgoal->f() << endl;
    update_h_static(open, opencheck, close);
    update_h_dynamic();

    path = vector<State>();
    while (s != start)
    {
        path.push_back(*s);
        s = s->parent;
    }
    start = sgoal;
    return !(start == goal);
}

vector<State> PLTASTAR::getPath()
{
    return path;
}

void PLTASTAR::setStartGoal(State s, State g, int bordw, int bordh)
{

    boardw = bordw;
    boardh = bordh;

    goalQ.setUpCompare(&compare1);

    start = new State_PLRTA(s);
    goal = new State_PLRTA(g);
    dummy = new State_PLRTA();
    dummy_static = new State_PLRTA_Static();

    goal->time = INT_MAX;
    goal->cost_d = DBL_MAX;
    goal->cost_s = DBL_MAX;
    start->time = 0;
    start->cost_d = 0;
    start->cost_s = 0;

    constructHtable();

    start->h_s = h_value(start);
    goal->h_s = 0;

    expandState.insert(goal);
    expandState.insert(start);
}

void PLTASTAR::setStatic(unordered_set<StaticObstacle> &s)
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

    dummy_static_obs.set(x, y);
    if (staticObstacles.find(dummy_static_obs) != staticObstacles.end())
        return 0;

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
    htable[getindex(goal->x, goal->y)] = 0;

    for (int i = 0; i < boardh; i++)
    {
        for (int j = 0; j < boardw; j++)
        {
            float a = abs(j - goal->x), b = abs(i - goal->y);
            htable[getindex(j, i)] = (a > b) ? a : b;
        }
    }
    htable_static = new float[boardw * boardh];
    memcpy(htable_static, htable, sizeof(float) * boardw * boardh);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "INITIAL HTABLE " << elapsed_secs << " SECS" << endl;
}
