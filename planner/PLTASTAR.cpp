
#include "PLTASTAR.h"
#include <iomanip>
#include <algorithm>
using namespace std;

#define DEBUG false
#define DEBUGCHILD false
#define DEBUGTABLE true


/**
 * Node-limited A∗ search toward the Goal
 * 
 * Helping function for calculating the probability of a dynamic obstacles crash with agent.
 * 
 * @param requestStart start state of the agent for next plan
 * @return dummy int
 */
int PLTASTAR::ASTAR(State requestStart)
{
   
    State_PLRTA *state;
    dummy->set(requestStart);
    start = expandState[dummy];

    //+++++++++++++++++++++++++++++++++++++++
    if (DEBUG && DEBUGTABLE)
    {
        printHStable(start->x, start->y);
        cout << "=================DDDDDD" << endl
             << endl;
        debug.clear();
        debug1.clear();
        debugstart = requestStart;
    }
    //++++++++++++++++++++++++++++++++++++++

    auto it = expandState.begin();

    while (it != expandState.end())
    {
        State_PLRTA *elem = it->key;
        ++it;
        if (elem->time < start->time)
        {
            expandState.erase(elem);
            delete elem;
        }
        else
        {
            elem->cost_s = DBL_MAX;
            elem->cost_d = DBL_MAX;
            elem->h_d = (elem->h_d > decay) ? elem->h_d - decay : 0;
            elem->h_s = h_value_static(elem);
        }
    }
    

    //clear open close & open check
    open.clear();
    open.setUpCompare(&compare);
    opencheck.clear();
    close.clear();

    start->cost_d = 0;
    start->cost_s = 0;
    start->depth = 0;
    start->h_s = h_value_static(start);

    int expansions = 0;
    //push start into open open check & expand state
    open.push(start);
    opencheck.insert(start);

    do
    {
        state = open.pop();
        if(max_time && max_time == state->time)
            break;
        if (DEBUG)
            cout << "\033[0;33m" 
                 << "STATES: "
                 << "\033[0;30m" << *state << " exp " << expansions << endl;
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
                    State_PLRTA *child_state = expandState[dummy];
                    if (!child_state)
                    {
                        child_state = new State_PLRTA(state->x + i, state->y + j, DBL_MAX, DBL_MAX, 0, 0, state->time + 1);
                        create_pred(child_state, state->time);
                        expandState.insert(child_state);
                    }
                    child_state->h_s = h_value_static(child_state);
                    double dcost = state->cost_d + cost_d(state,child_state);
                    double scost = state->cost_s + cost(state, child_state);

                    if (child_state->cost_s + child_state->cost_d > scost + dcost)
                    {

                        child_state->cost_s = scost;
                        child_state->cost_d = dcost;
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
                    if (DEBUG && DEBUGCHILD)
                        cout << "\033[0;35m"
                             << "CHILD: "
                             << "\033[0;30m" << *child_state << endl;
                }
            }
        }
    } while (expansions < LOOKAHEAD); //check collision for goal
    return 0;
}

/**
 * Learing step for the static heuristic
 * 
 * @param open1 priority Q for nodes have not been visited.
 * @param opencheck1 NON USED!!!! replace by opencheck in the function use to prevent duplicate nodes ignoring the time.
 * @param close1 HashSet used to recorde nodes that have been visited. 
 * @return dummy int
 */
int PLTASTAR::update_h_static(PQueue<State_PLRTA *> open1, HashSet<State_PLRTA *> opencheck1, HashSet<State_PLRTA *> close1)
{
    HashSet<State_PLRTA_Static *> close = HashSet<State_PLRTA_Static *>(&Hash_static, &equal_static, 0);
    HashSet<State_PLRTA_Static *> opencheck = HashSet<State_PLRTA_Static *>(&Hash_static, &equal_static, 0);
    PQueue<State_PLRTA_Static *> open;
    open.setUpCompare(&compare_static);
    open1.setUpCompare(&compare_static);

    for (auto it : close1)
    {
        dummy_static->set(it.key);
        State_PLRTA_Static *state = close[dummy_static];
        if (!state)
        {
            state = new State_PLRTA_Static(it.key);
            htable_static[getindex(state->x, state->y)] = DBL_MAX;
            for (auto i : it.key->pred)
                state->pred_static.emplace(i.x, i.y);
            close.insert(state);
        }
        else
        {
            for (auto i : it.key->pred)
                state->pred_static.emplace(i.x, i.y);
        }
    }

    while (!open1.empty())
    {
        State_PLRTA *state1 = open1.pop();
        dummy_static->set(state1);
        if (close.find(dummy_static))
        {
            if (dummy_static->x == goal->x && dummy_static->y == goal->y)
            {
                double cost_s = 0; //cost(dummy_static, dummy_static) + state1->h_s;
                int index = getindex(dummy_static->x, dummy_static->y);
                State_PLRTA_Static *state = close[dummy_static];
                for (auto i : state1->pred)
                    state->pred_static.emplace(i.x, i.y);

                if (htable_static[index] > cost_s)
                {
                    htable_static[index] = cost_s;
                    state->h_s = cost_s;
                }

                if (!opencheck.find(dummy_static))
                {
                    opencheck.insert(state);
                    open.push(state);
                }
                else
                {
                    open.moveUP(state->qindex);
                }
            }
        }
        else
        {
            State_PLRTA_Static *state = opencheck[dummy_static];
            if (!state)
            {
                state = new State_PLRTA_Static(state1);
                for (auto i : state1->pred)
                    state->pred_static.emplace(i.x, i.y);
                opencheck.insert(state);
                open.push(state);
            }
            else
            {
                for (auto i : state1->pred)
                    state->pred_static.emplace(i.x, i.y);
            }
        }
    }
    //open.reBuild();

    while (close.size() != 0 && !open.empty())
    {

        State_PLRTA_Static *state = open.pop();
        opencheck.erase(state);
        close.erase(state);
        //cerr << "pick state " <<state->x << " " << state->y<< " " <<" " << state->h_s <<  endl;
        // if(state->pred_static.size() == 0 || state->pred.size())
        // {
        //     cerr << "ERROR" << endl;
        // }

        for (point pred : state->pred_static)
        {
            dummy_static->set(pred);
            //cerr << "prePRED " <<pred.x << " " << pred.y<< " " <<" " << state->h_s <<  endl;
            State_PLRTA_Static *pred_state = close[dummy_static];
            if (pred_state)
            {
                double pred_h = cost(pred_state, state) + htable_static[getindex(state->x, state->y)];

                //cerr << "PRED "<<pred_state->x << " " << pred_state->y<< " " << htable_static[getindex(pred_state->x, pred_state->y)]<<" " << state->h_s <<  endl;
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

/**
 * Learing step for the dynamic heuristic
 * 
 * @return dummy int
 */
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

        for (point_t pred : state->pred)
        {
            dummy->set(pred);
            State_PLRTA *pred_state = close[dummy];
            if (pred_state)
            {
                double dcost = cost_d(pred_state,state) + state->h_d;
                // double dcost = cost_d(pred_state) + state->f();
                if (pred_state->h_d > dcost)
                {
                    pred_state->h_d = dcost;
                    // pred_state->h_d = dcost - pred_state->h_s - pred_state->cost_d - pred_state->cost_s;
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

    // for (auto i: expandState)
    //         cout << "\033[0;34m"
    //              << "UPOPEN: "
    //              << "\033[0;30m" << *(i.key) << endl;

    return 0;
}
/**
 * Tie-breaking method for picking the next state to go.
 * 
 * @return selected frontier state
 */
State_PLRTA *PLTASTAR::pickBest()
{
    double maxh = open.top()->f();

    if (DEBUG)
    {
        for (int i = 0; i < open.size(); i++)
            cout << "\033[0;34m"
                 << "INOPEN: "
                 << "\033[0;30m" << *(open[i]) << endl;
    }

    while (!open.empty() && open.top()->f() <= maxh)
    {
        if (DEBUG)
        {
            cout << "\033[0;31m"
                 << "CANDIT: "
                 << "\033[0;30m" << *(open.top()) << endl;
        }
        goalQ.push(open.pop());
    }

    State_PLRTA *s = NULL;
    do
    {
        //State_PLRTA *dd = goalQ.top();
        if (!s || s->depth < goalQ.top()->depth)
            s = goalQ.top();
        open.push(goalQ.pop());
    } while (!goalQ.empty());

    return s;
}

/**
 * Planning function call by executive for planning the next path.
 * 
 * @param requestStart start state of the agent for next plan
 * @return False if plan reach the goal
 */
int PLTASTAR::plan(State requestStart)
{
    startTime = requestStart.time;
    State_PLRTA *s, *sgoal;

    ASTAR(requestStart);

    if (open.empty())
        return 0;

    s = sgoal = pickBest();
    update_h_static(open, opencheck, close);
    update_h_dynamic();

    path = vector<State>();

    while (s != start)
    {
        if (DEBUG)
            cout << "\033[0;32m"
                 << "CHOOSE: "
                 << "\033[0;30m" << *s << endl;
        s->tempc = s->cost_d;
        path.push_back(*s);
        s = s->parent;
    }

    // start = sgoal;
    return !(start == goal);
}

/**
 * send start, end location and domain specification to planner
 * @param s set of static obstacles 
 */ 
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

    constructHtable(State(start->x, start->y), State(goal->x, goal->y));

    start->h_s = h_value(start);
    start->h_d = 0;
    goal->h_s = 0;
    goal->h_d = 0;

    expandState.insert(goal);
    expandState.insert(start);
}


/**
 * send Static Obstacles info to planner
 * @param s set of static obstacles 
 */ 
void PLTASTAR::setStatic(unordered_set<StaticObstacle> &s)
{
    staticObstacles = s;
}
