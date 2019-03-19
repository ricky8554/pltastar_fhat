#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include <string>
#include "obstacle.h"
#include <thread>
#include <mutex>

using namespace std;
int thread_number = 7;
Obstacle obstacle;
int boardh, boardw;
State goal;
State start;
void setObstacles(int argc, char *argv[])
{
    obstacle.setMode(0);
    if (argc > 1)
    {
        if (!strcmp(argv[1], "PLRTA_STAR") || !strcmp(argv[1], "1"))
            obstacle.setMode(1);
        else if (!strcmp(argv[1], "LSS_LRTA") || !strcmp(argv[1], "0"))
            obstacle.setMode(0);
        else if (!strcmp(argv[1], "LSS_LRTA_FHAT") || !strcmp(argv[1], "2"))
            obstacle.setMode(2);
        else if (!strcmp(argv[1], "PLRTA_STAR_FHAT") || !strcmp(argv[1], "3"))
            obstacle.setMode(3);
        else
            obstacle.setMode(0);
    }
    else
        obstacle.setMode(0);

    bool map = false;
    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-map"))
            map = true;
    }

    // double maxsp, minsp, r, x, y;
    // double x1, x2, x3, x4, y1, y2, y3, y4;
    // int numberofdynamic, numberofstatic;
    char c;
    string s;
    if (map)
    {
        cin >> s >> s >> s >> boardh >> s >> boardw >> s;
        cout << s << endl;
        cout << boardw << " " << boardw << endl;
        obstacle.setBoard(boardw, boardh);
        for (int i = 0; i < boardh; i++)
        {
            for (int j = 0; j < boardw; j++)
            {
                cin >> c;
                if (c == '@' || c == 'O' || c == 'T')
                    obstacle.addStaticObstacle(j, i);
                else if (c == '!')
                {
                    // obstacle.addDynamicObstacle(10, 10, 1, j, i);
                   
                }
                else if (c == 'S')
                {
                    obstacle.setStartPoint(j, i);
                    start = State(j, i,0,0);
                }
                else if (c == 'G')
                {
                    obstacle.setGoalPoint(j, i);
                    goal = State(j, i,0,0);
                }
            }
        }
    }
    else
    {
        int num_dynamic;
        cin >> boardw >> boardh >> num_dynamic;
        cout << boardw << " " << boardh << endl;
        obstacle.setBoard(boardw, boardh);
        for(int i = 0; i< num_dynamic; i++)
        {
            int dx, dy, instructions,steps;
            double heading, speed;
            cin >> dx >> dy >> instructions;
            DynamicObstacle d(dx,dy,instructions);
            for(int k = 0; k < instructions; k++)
            {
                cin >> heading >> speed >> steps;
                d.instructions.emplace_back(heading , speed , steps);
            }
            d.remain = d.instructions[0].steps;
            d.heading = d.instructions[0].heading;
            d.speed = d.instructions[0].speed;
            obstacle.addDynamicObstacle(d);
        }

        for (int i = boardh - 1; i >= 0; i--)
        {
            for (int j = 0; j < boardw; j++)
            {
                cin >> c;

                if (c == '#')
                    obstacle.addStaticObstacle(j, i);
                else if (c == '!')
                {
                    //obstacle.addDynamicObstacle(10, 10, 1, j, i);
                }
                else if (c == '@')
                {
                    obstacle.setStartPoint(j, i);
                    start = State(j, i,0,0);
                }
                else if (c == '*')
                {
                    obstacle.setGoalPoint(j, i);
                    goal = State(j, i,0,0);
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    int lookahead[5] = {10,30,100,300,1000};
    //int lookahead[9] = {150,160,170,180,190,155,165,175,185};
    setObstacles(argc, argv);

    vector<DynamicObstacle> dobs = obstacle.getDynamicObstacle1();
    for (int k = 1000; k <= 1000; k *= 10)
    {
        cerr << "MOVEMENT " << k<< endl;
        cout << "movements " << k << endl;
        for (int i = 0; i < 8; i++) //for (int i = 1; i < 4; i+=2)
        {
            cout << "algorithm " << i << endl;
            cerr << "algorithm " << i << endl;
            obstacle.setMode(i);
            for (int j = 0; j < 5; j++)
            {
                obstacle.setDynamicObstacle(dobs);
                obstacle.setStartPoint(start.x, start.y);
                obstacle.setGoalPoint(goal.x, goal.y);
                cout << lookahead[j]  << " " << obstacle.MoveObstacle(k, lookahead[j]) << endl;
            }
        }
    }
}

