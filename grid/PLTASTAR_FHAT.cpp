
#include "PLTASTAR_FHAT.h"
#include <iomanip>
#include <algorithm>
using namespace std;

int PLTASTAR_FHAT::ASTAR(State requestStart)
{
    State_PLRTA_FHAT *state;
    dummy->set(requestStart);
    start = expandState[dummy];

    //+++++++++++++++++++++++++++++++++++++++
    // printHtable(start->x, start->y);
    // cout << "=================DDDDDD" << endl
    //      << endl;
    // printDtable(start->x, start->y);
    // cout << "=================RRRR" << endl;
    // printDerrtable(start->x, start->y);
    // cout << "=================+++++++++++++" << endl;
    // debug.clear();
    // debug1.clear();
    // debugstart = requestStart;
    //++++++++++++++++++++++++++++++++++++++

    auto it = expandState.begin();

    while (it != expandState.end())
    {
        State_PLRTA_FHAT *elem = it->key;
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
            elem->derr = getDerr(elem);
            elem->d = getD(elem);
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
    start->derr = getDerr(start);
    start->d = getD(start);

    int expansions = 0;
    //push start into open open check & expand state
    open.push(start);
    opencheck.insert(start);
    double dsum = 0, hsum = 0;

    // cout << "\033[0;31m" << "DERR " << derr  << " HERR " << herr << "\033[0;30m" << endl;

    do
    {
        state = open.pop();

        // cout << "\033[0;33m" << "STATE: " << "\033[0;30m" << *state << endl;

        opencheck.erase(state);
        close.insert(state);
        expansions += 1;
        State_PLRTA_FHAT *best_child = NULL;

        debug.emplace(state->x, state->y);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                dummy->set(state->x + i, state->y + j, state->time + 1);
                if (checkValid(dummy))
                {
                    State_PLRTA_FHAT *child_state = expandState[dummy];
                    if (!child_state || !close.find(dummy))
                    {
                        if (!child_state)
                        {
                            child_state = new State_PLRTA_FHAT(state->x + i, state->y + j, DBL_MAX, DBL_MAX, 0, 0, state->time + 1);
                            child_state->d = getD(child_state);
                            child_state->derr = getDerr(child_state);
                            create_pred(child_state, state->time);
                            expandState.insert(child_state);
                        }
                        child_state->h_s = h_value_static(child_state);
                        double dcost = state->cost_d + cost_d(child_state);
                        double scost = state->cost_s + cost(state, child_state);

                        if (child_state->cost_s + child_state->cost_d > scost + dcost)
                        {

                            child_state->cost_s = scost;
                            child_state->cost_d = dcost;
                            child_state->parent = state;
                            child_state->depth = state->depth + 1;
                            child_state->h_error = herr * (child_state->derr / (1 - derr));
                            
                            //+++++++++++++++++++++++++++++++++++++++
                            if(isGoal(child_state) && dcost < 20 && (touch_goal == NULL || touch_goal->depth < child_state->depth))
                            {
                                touch_goal = child_state;
                            }
                            //+++++++++++++++++++++++++++++++++++++++

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
                    }

                    // cout << "\033[0;35m" << "CHILD: " << "\033[0;30m" << *child_state << endl;
                    if (!best_child || best_child->f_static() > child_state->f_static())
                        best_child = child_state;
                }
            }
        }

        if (best_child)
        {
            double fdiff = best_child->f_static() - state->f_static();
            fdiff = (fdiff < 0) ? 0 : fdiff;
            hsum += fdiff;

            double ddiff = best_child->d + 1 - state->d;
            //double ddiff = getDistance(best_child) + 1 - getDistance(state);
            if(state->x == goal_plan.x && state->y == goal_plan.y)
                ddiff = 0;
            // if (best_child->x == state->x && best_child->y == state->y)
            //     ddiff = 0;
            // else
            //     ddiff = best_child->d + 1 - state->d;
            ddiff = (ddiff < 0) ? 0 : ddiff;
            ddiff = (ddiff > 0.99999) ? 0.99999 : ddiff;
            dsum += ddiff;

            // if(dsum != 0)
            //   cout << "warning" << endl;

            // if(hsum != 0)
            //   cout << "warning1" << endl;
        }

    } while (expansions < LOOKAHEAD); //check collision for goal

    derr = dsum / expansions;
    herr = hsum / expansions;
    // derr = (derr + dsum / expansions)/2;
    // herr = (herr + hsum / expansions)/2;
    return 0;
}

int PLTASTAR_FHAT::update_h_static(PQueue<State_PLRTA_FHAT *> open1, HashSet<State_PLRTA_FHAT *> opencheck1, HashSet<State_PLRTA_FHAT *> close1)
{
    HashSet<State_PLRTA_FHAT_Static *> close = HashSet<State_PLRTA_FHAT_Static *>(&Hash_static, &equal_static, 0);
    HashSet<State_PLRTA_FHAT_Static *> opencheck = HashSet<State_PLRTA_FHAT_Static *>(&Hash_static, &equal_static, 0);
    PQueue<State_PLRTA_FHAT_Static *> open;
    open.setUpCompare(&compare_static);
    open1.setUpCompare(&compare_static);
    
    for (auto it : close1)
    {
        dummy_static->set(it.key);
        State_PLRTA_FHAT_Static *state = close[dummy_static];
        if (!state)
        {
            state = new State_PLRTA_FHAT_Static(it.key);
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
        State_PLRTA_FHAT *state1 = open1.pop();
        dummy_static->set(state1);
        if (close.find(dummy_static))
        {
            if (dummy_static->x == goal->x && dummy_static->y == goal->y)
            {
                double cost_s = 0;
                int index = getindex(dummy_static->x, dummy_static->y);
                State_PLRTA_FHAT_Static *state = opencheck[dummy_static];
                if(!state)
                    state = close[dummy_static];

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
            State_PLRTA_FHAT_Static *state = opencheck[dummy_static];
            if (!state)
            {
                state = new State_PLRTA_FHAT_Static(state1);
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

    // open.reBuild();

    while (close.size() != 0 && !open.empty())
    {

        State_PLRTA_FHAT_Static *state = open.pop();
        int index1 = getindex(state->x, state->y);
        state->derr = derrtable[index1]; // consider remove this
        state->d = dtable[index1];       // consider remove this
        opencheck.erase(state);
        close.erase(state);

        for (point pred : state->pred_static)
        {
            dummy_static->set(pred);
            State_PLRTA_FHAT_Static *pred_state = close[dummy_static];
            if (pred_state)
            {
                State_PLRTA_FHAT_Static *temp = opencheck[dummy_static];
                if(temp)
                    pred_state = temp;

                double pred_h = cost(pred_state, state) + htable_static[index1];
                int index = getindex(pred_state->x, pred_state->y);
                if (htable_static[index] > pred_h)
                {
                    htable_static[index] = pred_state->h_s = pred_h;
                    int add = (state->x == pred.x && state->y == pred.y) ? 0 : 1;
                    derrtable[index] = state->derr; //change
                    dtable[index] = state->d + add; //change

                    if (!temp)
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

int PLTASTAR_FHAT::update_h_dynamic()
{
    for (auto it : close)
    {
        it.key->h_d = DBL_MAX;
    }

    open.setUpCompare(&compare2);

    while (close.size() != 0 && !open.empty())
    {
        State_PLRTA_FHAT *state = open.pop();
        opencheck.erase(state);
        close.erase(state);

        for (point_t pred : state->pred)
        {
            dummy->set(pred);
            State_PLRTA_FHAT *pred_state = close[dummy];
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

State_PLRTA_FHAT *PLTASTAR_FHAT::pickBest()
{
    double maxh = open.top()->fhat();

    // for(int i = 0; i< open.size(); i++)
    //     cout << "\033[0;34m" << "INOPEN: " << "\033[0;30m" << *(open[i]) << endl;

    while (!open.empty() && open.top()->fhat() <= maxh)
    {
        // cout << "\033[0;31m" << "CANDIT: " << "\033[0;30m" << *(open.top()) << endl;
        goalQ.push(open.pop());
    }

    State_PLRTA_FHAT *s = NULL;
    do
    {
        if (!s || s->depth < goalQ.top()->depth)
            s = goalQ.top();
        open.push(goalQ.pop());
    } while (!goalQ.empty());
    // cerr << s->x << " " << s->y << " " << s->time << " " << s->depth << " final" << endl;

    return s;
}

int PLTASTAR_FHAT::plan(State requestStart)
{
    startTime = requestStart.time;
    State_PLRTA_FHAT *s, *sgoal;

    ASTAR(requestStart);

    if (open.empty())
        return 0;

    s = sgoal = pickBest();
    update_h_static(open, opencheck, close);
    update_h_dynamic();

    path = vector<State>();
    //cerr << "CHOSE: " << s->x << " " << s->y << " " << s->cost_s << " " << s->cost_d << " " << s->h_s << " " << s->h_d << " " << s->f() << " " << s->time << endl;

    bool reach_goal = false;
    

    while (s != start)
    {
        // cout << "\033[0;32m" << "CHOOSE: " << "\033[0;30m" << *s << endl;
        //+++++++++++++++
        if(isGoal(s))
            reach_goal = true;
        //+++++++++++++++

        path.push_back(*s);
        s = s->parent;
    }

    //+++++++++++++++
    if( !reach_goal && touch_goal != NULL)
    {
        s = sgoal = touch_goal;
        path.clear();
        while (s != start)
        {
            path.push_back(*s);
            s = s->parent;
        }
    }
    touch_goal = NULL;
    //+++++++++++++++
    

    // start = sgoal;
    return !(start == goal);
}

void PLTASTAR_FHAT::setStartGoal(State s, State g, int bordw, int bordh)
{

    boardw = bordw;
    boardh = bordh;

    goalQ.setUpCompare(&compare1);

    start = new State_PLRTA_FHAT(s);
    goal = new State_PLRTA_FHAT(g);
    dummy = new State_PLRTA_FHAT();
    dummy_static = new State_PLRTA_FHAT_Static();

    start->d = start->derr = getDistance(start);
    goal->d = goal->derr = getDistance(goal);

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

void PLTASTAR_FHAT::setStatic(unordered_set<StaticObstacle> &s)
{
    staticObstacles = s;
}

// void PLTASTAR_FHAT::setDynamic(vector<DynamicObstacle> &d)
// {
//     dynamicObstacles = d;

//     for (int i = 0; i < dynamicObstacles.size(); i++)
//     {
//         int x = dynamicObstacles[i].x, y = dynamicObstacles[i].y;
//         int x1 = x;

//         if(x1 == boardw - 1)
//         {
//             dynamicObstacles[i].right = boardw - 1;
//         }
//         else
//         {
//             dummy_static_obs.set(x1, y);
//             while (staticObstacles.find(dummy_static_obs) == staticObstacles.end() && x1 <= boardw - 1)
//             {
//                 x1 += 1;
//                 dummy_static_obs.set(x1, y);
//             }

//             dynamicObstacles[i].right = x1 - 1;
//         }


//         x1 = x;
        
//         if(x1 == 0)
//         {
//             dynamicObstacles[i].left = 0;
//         }
//         else
//         {
//             dummy_static_obs.set(x1, y);
//             while (staticObstacles.find(dummy_static_obs) == staticObstacles.end() && x1 >= 0)
//             {
//                 x1 -= 1;
//                 dummy_static_obs.set(x1, y);
//             }
//             dynamicObstacles[i].left = x1 + 1;
//         }
//     }
// }

double PLTASTAR_FHAT::getDerr(State_PLRTA_FHAT *s)
{
    return derrtable[getindex(s->x, s->y)];
}

double PLTASTAR_FHAT::getD(State_PLRTA_FHAT *s)
{
    return dtable[getindex(s->x, s->y)];
}
