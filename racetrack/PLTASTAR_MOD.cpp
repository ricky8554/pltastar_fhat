
#include "PLTASTAR_MOD.h"
#include <iomanip>
#include <algorithm>
using namespace std;

#define DEBUG false
#define DEBUGCHILD false
#define DEBUGTABLE true

int PLTASTAR_MOD::ASTAR(State requestStart)
{
    State_PLRTA_MOD *state;
    dummy->set(requestStart);
    start = expandState[dummy];

    //+++++++++++++++++++++++++++++++++++++++
    if (DEBUG && DEBUGTABLE)
    {
        printHtable(start->x, start->y);
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
        State_PLRTA_MOD *elem = it->key;
        ++it;
        if (elem->time < start->time)
        {
            expandState.erase(elem);
            delete elem;
        }
        else
        {
            elem->pred.clear();
            elem->cost_s = DBL_MAX;
            elem->cost_d = DBL_MAX;
            // elem->h_d = (elem->h_d > decay) ? elem->h_d - decay : 0;
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
        if (DEBUG)
            cout << "\033[0;33m"
                 << "STATES: "
                 << "\033[0;30m" << *state << endl;
        opencheck.erase(state);
        close.insert(state);
        expansions += 1;
        debug.emplace(state->x, state->y,0,0);
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int dx = state->dx + i, dy = state->dy + j;
                dummy->set(state->x + dx, state->y + dy, dx, dy, state->time + 1);

                if (!checkValid(state, dummy))
                    find_avaliable_state(state, dummy, dx, dy);

                State_PLRTA_MOD *child_state = expandState[dummy];

                if (!close.find(dummy))
                {
                    if (!child_state)
                    {
                        child_state = new State_PLRTA_MOD(dummy->x, dummy->y, dx, dy, DBL_MAX, DBL_MAX, 0, 0, state->time + 1);
                        expandState.insert(child_state);
                    }

                    child_state->h_s = h_value_static(child_state);
                    double dcost = state->cost_d + cost_d(state, child_state);
                    double scost = state->cost_s + cost(state, child_state);

                    if (child_state->cost_s + child_state->cost_d > scost + dcost)
                    {
                        child_state->cost_s = scost;
                        child_state->cost_d = dcost;
                        child_state->parent = state;
                        child_state->depth = state->depth + 1;

                        if (!opencheck.find(child_state))
                        {
                            opencheck.insert(child_state);
                            debug1.emplace(child_state->x, child_state->y, 0, 0);
                            open.push(child_state);
                        }
                        else
                        {
                            open.moveUP(child_state->qindex);
                        }
                    }
                }
                if(child_state->h_s >100000000)
                    exit(0);
                child_state->pred.emplace(state->x, state->y, state->dx, state->dy, state->time);
            }
        }
    } while (expansions < LOOKAHEAD); //check collision for goal
    return 0;
}

int PLTASTAR_MOD::update_h_static(PQueue<State_PLRTA_MOD *> open1, HashSet<State_PLRTA_MOD *> opencheck1, HashSet<State_PLRTA_MOD *> close1)
{
    HashSet<State_PLRTA_MOD_Static *> close = HashSet<State_PLRTA_MOD_Static *>(&Hash_static, &equal_static, 0);
    HashSet<State_PLRTA_MOD_Static *> opencheck = HashSet<State_PLRTA_MOD_Static *>(&Hash_static, &equal_static, 0);
    PQueue<State_PLRTA_MOD_Static *> open;
    open.setUpCompare(&compare_static);
    open1.setUpCompare(&compare_static);

    for (auto it : close1)
    {
        dummy_static->set(it.key);
        State_PLRTA_MOD_Static *state = close[dummy_static];
        if (!state)
        {
            state = new State_PLRTA_MOD_Static(it.key);
            set_h_value_static(state,DBL_MAX);
            for (auto i : it.key->pred)
                state->pred_static.emplace(i.x, i.y,i.dx,i.dy);
            close.insert(state);
        }
        else
        {
            for (auto i : it.key->pred)
                state->pred_static.emplace(i.x, i.y,i.dx,i.dy);
        }
    }

    while (!open1.empty())
    {
        State_PLRTA_MOD *state1 = open1.pop();
        dummy_static->set(state1);
        if (close.find(dummy_static))
        {
            if (dummy_static->x == goal->x && dummy_static->y == goal->y)
            {
                double cost_s = 0; //cost(dummy_static, dummy_static) + state1->h_s;
                State_PLRTA_MOD_Static *state = close[dummy_static];
                for (auto i : state1->pred)
                    state->pred_static.emplace(i.x, i.y,i.dx,i.dy);

                if ( h_value_static(state) > cost_s)
                {
                    set_h_value_static(state,cost_s);
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
            else
            {
                State_PLRTA_MOD_Static *state = close[dummy_static];
                for (auto i : state1->pred)
                {
                    state->pred_static.emplace(i.x, i.y,i.dx,i.dy);
                }
            }
        }
        else
        {
            State_PLRTA_MOD_Static *state = opencheck[dummy_static];
            if (!state)
            {
                state = new State_PLRTA_MOD_Static(state1);
                for (auto i : state1->pred)
                    state->pred_static.emplace(i.x, i.y,i.dx,i.dy);
                opencheck.insert(state);
                open.push(state);
            }
            else
            {
                for (auto i : state1->pred)
                    state->pred_static.emplace(i.x, i.y,i.dx,i.dy);
            }
        }
    }
    //open.reBuild();

    while (close.size() != 0 && !open.empty())
    {

        State_PLRTA_MOD_Static *state = open.pop();
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
            State_PLRTA_MOD_Static *pred_state = close[dummy_static];
            if (pred_state)
            {
                double pred_h = cost(pred_state, state) + h_value_static(state);

                //cerr << "PRED "<<pred_state->x << " " << pred_state->y<< " " << htable_static[getindex(pred_state->x, pred_state->y)]<<" " << state->h_s <<  endl;
                if (h_value_static(pred_state) > pred_h)
                {
                    pred_state->h_s = pred_h;
                    set_h_value_static(pred_state,pred_h); 

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

//currently use h_d + h_s as based, consider swich to
int PLTASTAR_MOD::update_h_dynamic()
{
    for (auto it : close)
    {
        State_PLRTA_MOD *p = it.key;
        p->h_d = DBL_MAX;
        p->h_s = h_value_static(p);
    }

    for (int i = 0; i < open.size(); i++)
    {
        State_PLRTA_MOD *p = open[i];
        p->h_s = h_value_static(p);
    }

    // open.setUpCompare(&compare2);
    open.setUpCompare(&compare);

    while (close.size() != 0 && !open.empty())
    {
        State_PLRTA_MOD *state = open.pop();
        opencheck.erase(state);
        close.erase(state);

        for (point_t pred : state->pred)
        {
            dummy->set(pred);
            State_PLRTA_MOD *pred_state = close[dummy];
            if (pred_state)
            {
                // double dcost = cost_d(pred_state) + state->h_d;
                double dcost = cost_d(pred_state,state) + state->f();
                if (pred_state->f() > dcost)
                {
                    // pred_state->h_d = dcost;
                    pred_state->h_d = dcost - pred_state->h_s - pred_state->cost_d - pred_state->cost_s;
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

State_PLRTA_MOD *PLTASTAR_MOD::pickBest()
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

    State_PLRTA_MOD *s = NULL;
    do
    {
        //State_PLRTA_MOD *dd = goalQ.top();
        if (!s || s->depth < goalQ.top()->depth)
            s = goalQ.top();
        open.push(goalQ.pop());
    } while (!goalQ.empty());

    return s;
}

int PLTASTAR_MOD::plan(State requestStart)
{
    startTime = requestStart.time;
    State_PLRTA_MOD *s, *sgoal;

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

void PLTASTAR_MOD::setStartGoal(State s, State g, int bordw, int bordh)
{

    boardw = bordw;
    boardh = bordh;

    goalQ.setUpCompare(&compare1);

    start = new State_PLRTA_MOD(s);
    goal = new State_PLRTA_MOD(g);
    dummy = new State_PLRTA_MOD();
    dummy_static = new State_PLRTA_MOD_Static();

    goal->time = INT_MAX;
    goal->cost_d = DBL_MAX;
    goal->cost_s = DBL_MAX;
    start->time = 0;
    start->cost_d = 0;
    start->cost_s = 0;

    constructHtable(State(start->x, start->y,0,0), State(goal->x, goal->y,0,0));

    start->h_s = h_value(start);
    start->h_d = 0;
    goal->h_s = 0;
    goal->h_d = 0;

    expandState.insert(goal);
    expandState.insert(start);
}

void PLTASTAR_MOD::setStatic(unordered_set<StaticObstacle> &s)
{
    staticObstacles = s;
}
