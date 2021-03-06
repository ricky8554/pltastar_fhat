
#include "LSS_LRTA.h"
#include <iomanip>
#include <algorithm>
using namespace std;


int Lss_Lrta::ASTAR(State requestStart)
{

    State_LSS *state;
    dummy->set(requestStart);
    start = expandState[dummy];

    //+++++++++++++++++++++++++++++++++++++++
    // printHtable(start->x, start->y);
    // cout << "=================DDDDDD" << endl
    //      << endl;
    // debug.clear();
    // debug1.clear();
    // debugstart = requestStart;
    //++++++++++++++++++++++++++++++++++++++

    auto it = expandState.begin();

    while (it != expandState.end())
    {
        State_LSS *elem = it->key;
        ++it;

        if (elem->time < start->time)
        {
            expandState.erase(elem);
            delete elem;
        }
        else
        {
            elem->cost = DBL_MAX;
        }
    }

    //clear open close & open check
    open.clear();
    open.setUpCompare(&compare);
    opencheck.clear();
    close.clear();

    start->cost = 0;
    start->depth = 0;

    int expansions = 0;
    //push start into open open check & expand state
    open.push(start);
    opencheck.insert(start);

    do
    {

        state = open.pop();
        if(max_time && max_time == state->time)
            break;
        // cout << "\033[0;33m" << "STATE: " << "\033[0;30m" << *state << endl;

        opencheck.erase(state);
        close.insert(state);
        expansions += 1;
        
        debug.emplace(state->x, state->y);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                
                dummy->set(state->x + i, state->y + j, state->time + 1);
                if (checkValid(dummy) && !close.find(dummy))
                {
                    State_LSS *child_state = expandState[dummy];

                    if (!child_state)
                    {
                        child_state = new State_LSS(state->x + i, state->y + j, DBL_MAX, h_value(dummy), state->time + 1);
                        create_pred(child_state,state->time);
                        expandState.insert(child_state);
                    }
                    double next_cost = state->cost + cost(state, child_state) + cost_d(state,child_state);

                    if (child_state->cost > next_cost)
                    {
                        child_state->cost = next_cost;
                        child_state->parent = state;
                        child_state->depth = state->depth + 1;
                        

                        if (!opencheck.find(child_state))
                        {
                            debug1.emplace(child_state->x, child_state->y);
                            opencheck.insert(child_state);
                            open.push(child_state);
                        }
                        else
                        {
                            open.moveUP(child_state->qindex);
                        }
                    }

                    // cout << "\033[0;35m" << "CHILD: " << "\033[0;30m" << *child_state << endl;
                }
            }
        }
    } while (expansions < LOOKAHEAD);
   
    return 0;
}

int Lss_Lrta::update_h()
{
    
    for (auto it : close)
    {
        it.key->h = DBL_MAX;
    }
    open.setUpCompare(&compare1);

    while (close.size() != 0 && open.size() != 0)
    {

        State_LSS *state = open.pop();
        opencheck.erase(state);
        close.erase(state);

        for (point_t pred : state->pred)
        {
            dummy->set(pred);
            State_LSS *pred_state = close[dummy];
            if (pred_state)
            {
                double pred_h = cost(pred_state, state) + state->h + cost_d(pred_state,state);
                if (pred_state->h > pred_h)
                {
                    pred_state->h = pred_h;
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

State_LSS *Lss_Lrta::pickBest()
{
    double maxh = open.top()->f();
    // for(int i = 0; i< open.size(); i++)
    //     cout << "\033[0;34m" << "INOPEN: " << "\033[0;30m" << *(open[i]) << endl;
    
    while (!open.empty() && open.top()->f() <= maxh)
    {
        // cout << "\033[0;31m" << "CANDIT: " << "\033[0;30m" << *(open.top()) << endl;
        goalQ.push(open.pop());
    }

    open.setUpCompare(&compare1);
    State_LSS *s = NULL;
    do
    {
        if(!s || s->depth < goalQ.top()->depth)
            s = goalQ.top();
        open.push(goalQ.pop());
    } while (!goalQ.empty());

    return s;
}

int Lss_Lrta::plan(State requestStart)
{
    startTime = requestStart.time;
    State_LSS *s, *sgoal;
    
    ASTAR(requestStart);
    
    if (open.empty())
        return 0;
    s = sgoal = pickBest();

    update_h();
    
    path = vector<State>();
    while (s != start)
    {
        // cout << "\033[0;32m" << "CHOSE: " << "\033[0;30m" << *s << endl;
        path.push_back(*s);
        s = s->parent;
    }
    
    start = sgoal;
    return !(start == goal);
}


void Lss_Lrta::setStartGoal(State s, State g, int bordw, int bordh)
{
    boardw = bordw;
    boardh = bordh;

    goalQ.setUpCompare(&compare1);

    start = new State_LSS(s);
    goal = new State_LSS(g);
    dummy = new State_LSS();

    goal->time = INT_MAX;
    start->time = 0;
    goal->cost = DBL_MAX;
    start->cost = 0;

    constructHtable(State(start->x,goal->y),State(goal->x,goal->y));

    start->h = h_value(start);
    goal->h = 0;

    expandState.insert(goal);
    expandState.insert(start);
}

void Lss_Lrta::setStatic(unordered_set<StaticObstacle> &s)
{
    staticObstacles = s;
}



