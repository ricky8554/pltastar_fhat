
#include "LSS_LRTA.h"
#include <iomanip>
#include <algorithm>
using namespace std;

int Lss_Lrta::ASTAR(State requestStart)
{
    
    State_LSS *state;

    dummy->set(requestStart);
    
    
    start = expandState[dummy];
    
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
        cerr << "state " << state->x << " " << state->y << " " << state->dx << " " << state->dy << " " << state->time << " " << state->f() << " " << endl;
        cerr << "COMPARE " << getDistance(state) << " " << state->h << endl;
        opencheck.erase(state);
        close.insert(state);
        expansions += 1;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int dx = state->dx + i, dy = state->dy + j;
                dummy->set(state->x + dx, state->y + dy,dx,dy ,state->time + 1);
                int valid = checkValid(state, dummy);
                // std::cerr <<"expanded "  <<state->x << " " <<state->y  <<" "<<dummy->x<< " " <<dummy->y  <<" " << bo <<endl;
                if (!valid)
                {
                    int cx = state->x, cy = state->y, fx = dummy->x, fy = dummy->y;
                    int distance = sqrt( (cx-fx)*(cx-fx) + (cy-fy)*(cy-fy) );
                    int factor1 = int(distance / 0.05);
                    double factor = factor1;
                    double diffx = (fx - cx) / factor, diffy = (fy - cy) / factor;
                    int finalx = cx, finaly = cy;
                        
                    for (int i = 0; i < factor1; i++)
                    {
                        finalx += diffx;
                        finaly += diffy;
                        if(!checkValid2(finalx,finaly))
                        {
                            finalx -= diffx;
                            finaly -= diffy;
                            break;
                        }
                            
                    }
                    finalx = round(finalx);
                    finaly = round(finaly);
                    dx = dy = 0;
                    dummy->set(finalx, finaly,dx,dy ,state->time + 1);
                }

                State_LSS *child_state = expandState[dummy];

                if (close.find(dummy))
                {
                    child_state->pred.emplace(state->x, state->y, state->dx, state->dy, state->time);
                    continue;
                }
                

                if (!child_state)
                {
                    child_state = new State_LSS(dummy->x, dummy->y, dx, dy, DBL_MAX, h_value(dummy), state->time + 1);
                    expandState.insert(child_state);
                }

                child_state->pred.emplace(state->x, state->y, state->dx, state->dy, state->time);

                double next_cost = state->cost + cost(state, child_state) + cost_d(state, child_state);

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
                double pred_h = cost(pred_state, state) + state->h + cost_d(pred_state, state);
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
    while (!open.empty() && open.top()->f() <= maxh)
    {
        goalQ.push(open.pop());
    }

    open.setUpCompare(&compare1);
    State_LSS *s = NULL;
    do
    {
        if (!s || s->depth < goalQ.top()->depth)
            s = goalQ.top();
        open.push(goalQ.pop());
    } while (!goalQ.empty());

    return s;
}

int Lss_Lrta::plan(State requestStart)
{
    cerr << "START" << endl;
    startTime = requestStart.time;
    State_LSS *s, *sgoal;


    ASTAR(requestStart);

    if (open.empty())
        return 0;
    s = sgoal = pickBest();

    //std::cout << "CHOSE: " << sgoal->x << "\t" << sgoal->y << "\t" << sgoal->cost << "\t" << sgoal->h << "\t" << sgoal->f() << endl;
    update_h();

    path = vector<State>();
    while (s != start)
    {
        path.push_back(*s);
        s = s->parent;
    }

    start = sgoal;
    cerr << "END" << endl;
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

    constructHtable(State(start->x, goal->y, 0, 0), State(goal->x, goal->y, 0, 0));
    
    start->h = h_value(start);
    goal->h = 0;

    expandState.insert(goal);
    expandState.insert(start);
}

void Lss_Lrta::setStatic(unordered_set<StaticObstacle> &s)
{
    staticObstacles = s;
}

void Lss_Lrta::setDynamic(vector<DynamicObstacle> &d)
{
    dynamicObstacles = d;
}
