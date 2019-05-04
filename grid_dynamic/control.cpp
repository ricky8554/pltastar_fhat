#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include <string>
#include "obstacle.h"
#include <thread>
#include <vector>
#include <mutex>

using namespace std;
int thread_number = 8;
vector<Obstacle> obstacle;
int boardh, boardw;
string filename = "temp";
State goal;
State start;
int Movement = 500;
mutex mutex_out, mutex_generator, mutex_initial, mutex_err;

void setObstacles(int argc, char *argv[])
{

    for (int i = 0; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-movement"))
            Movement = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "-thread"))
            thread_number = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "-file"))
            filename = argv[i + 1];
    }
    obstacle =  vector<Obstacle>(thread_number); 
    for (auto &obs : obstacle)
        obs.setMode(0);

    // double maxsp, minsp, r, x, y;
    // double x1, x2, x3, x4, y1, y2, y3, y4;
    // int numberofdynamic, numberofstatic;
    char c;
    string s;

    int num_dynamic;
    cin >> boardw >> boardh >> num_dynamic;
    for (auto &obs : obstacle)
        obs.setBoard(boardw, boardh);
    for (int i = 0; i < num_dynamic; i++)
    {
        int dx, dy, instructions, steps;
        double heading, speed;
        cin >> dx >> dy >> instructions;
        DynamicObstacle d(dx, dy, instructions);
        for (int k = 0; k < instructions; k++)
        {
            cin >> heading >> speed >> steps;
            d.instructions.emplace_back(heading, speed, steps);
        }
        d.remain = d.instructions[0].steps;
        d.heading = d.instructions[0].heading;
        d.speed = d.instructions[0].speed;
        for (auto &obs : obstacle)
            obs.addDynamicObstacle(d);
    }

    for (int i = boardh - 1; i >= 0; i--)
    {
        for (int j = 0; j < boardw; j++)
        {
            cin >> c;

            if (c == '#')
                for (auto &obs : obstacle)
                    obs.addStaticObstacle(j, i);
            else if (c == '!')
            {
                //obstacle.addDynamicObstacle(10, 10, 1, j, i);
            }
            else if (c == '@')
            {
                for (auto &obs : obstacle)
                    obs.setStartPoint(j, i);
                start = State(j, i);
            }
            else if (c == '*')
            {
                for (auto &obs : obstacle)
                    obs.setGoalPoint(j, i);
                goal = State(j, i);
            }
        }
    }
}

struct generator_t
{
    const int algo_max = 12;
    const int lookahead_index_max = 7;
    int lookahead[7] = {1, 3, 10, 30, 100, 300, 1000};
    int movement = 500;
    int initial_alog = 1;
    int algorithm = initial_alog;
    int lookahead_index = lookahead_index_max - 1;
    int operator()(int &move, int &algo, int &look)
    {
        mutex_generator.lock();
        if (lookahead_index == -1)
        {
            mutex_generator.unlock();
            return 0;
        }
        move = movement;
        algo = algorithm++;
        // while (algorithm != 1 && algorithm != 3 && algorithm != 12 && algorithm != 13 && algorithm != algo_max)
        while (algorithm != 1 && algorithm != 3 &&  algorithm != 7 && algorithm != 9 && algorithm != algo_max)
            algorithm++;
        look = lookahead[lookahead_index];
        if (algorithm == algo_max)
        {
            algorithm = 1;
            lookahead_index -= 1;
        }
        mutex_generator.unlock();
        return 1;
    }
} generator;

void run_thread(int index)
{
    int algo, move, look, cost;
    mutex_initial.lock();
    int sx = start.x, sy = start.y, gx = goal.x, gy = goal.y;
    mutex_initial.unlock();
    vector<DynamicObstacle> dobs = obstacle[index].getDynamicObstacle1();
    while (generator(move, algo, look))
    {
        mutex_err.lock();
        cerr << "start movements " << move << " algorithm " << algo << " lookahead " << look << " thread " << index << endl;
        obstacle[index].setMode(algo);
        obstacle[index].setDynamicObstacle(dobs);
        obstacle[index].setStartPoint(sx, sy);
        obstacle[index].setGoalPoint(gx, gy);
        mutex_err.unlock();
        cost = obstacle[index].MoveObstacle(move, look);
        mutex_out.lock();
        cout << "movements " << move << " algorithm " << algo << " lookahead " << look << " cost " << cost << " file " << filename << endl;
        mutex_out.unlock();
    }
}

int main(int argc, char *argv[])
{
    setObstacles(argc, argv);
    generator.movement = Movement;
    vector<thread> threads;
    for (int i = 0; i < thread_number; i++)
        threads.push_back(thread([=] { run_thread(i); }));
    for (thread &t : threads)
        t.join();
}

// // int lookahead[5] = {10,30,100,300,1000};
// // int lookahead[5] = {1,3,5,7,10};
// // setObstacles(argc, argv);

// // vector<DynamicObstacle> dobs = obstacle.getDynamicObstacle1();
// // for (int k = 1000; k <= 1000; k *= 10)
// // {
// //     cerr << "MOVEMENT " << k << endl;
// //     cout << "movements " << k << endl;
// //     for (int i = 0; i < 2; i++) //for (int i = 1; i < 4; i+=2)
// //     {
// //         int alg = i % 2 ? 1 : 4;
// //         cout << "algorithm " << alg << endl;
// //         cerr << "algorithm " << alg << endl;
// //         obstacle.setMode(alg);
// //         for (int j = 1; j < 11; j++)
// //         {
// //         obstacle.setDynamicObstacle(dobs);
// //         obstacle.setStartPoint(start.x, start.y);
// //         obstacle.setGoalPoint(goal.x, goal.y);
// //         // cout << lookahead[j] << " " << obstacle.MoveObstacle(k, lookahead[j]) << endl;
// //         cout << j << " " << obstacle.MoveObstacle(k, j) << endl;
// //         }
// //     }
// // }

// #include <cstdlib>
// #include <cstdio>
// #include <iostream>
// #include <fstream>
// #include <cmath>
// #include <stdio.h>
// #include <string>
// #include "obstacle.h"
// #include <thread>
// #include <mutex>

// using namespace std;
// int thread_number = 7;
// Obstacle obstacle;
// int boardh, boardw;
// State goal;
// State start;
// int Movement = 1000;
// void setObstacles(int argc, char *argv[])
// {
//     obstacle.setMode(0);
//     if (argc > 1)
//     {
//         Movement = atoi(argv[1]);
//     }

//     obstacle.setMode(0);

//     bool map = false;
//     for (int i = 1; i < argc; i++)
//     {
//         if (!strcmp(argv[i], "-map"))
//             map = true;
//     }

//     // double maxsp, minsp, r, x, y;
//     // double x1, x2, x3, x4, y1, y2, y3, y4;
//     // int numberofdynamic, numberofstatic;
//     char c;
//     string s;

//     cin >> boardw >> boardh;
//     obstacle.setBoard(boardw, boardh);
//     for (int i = boardh - 1; i >= 0; i--)
//     {
//         for (int j = 0; j < boardw; j++)
//         {
//             cin >> c;

//             if (c == '#')
//                 obstacle.addStaticObstacle(j, i);
//             else if (c == '_' || c == '-')
//             {
//             }
//             else if (c == '@')
//             {
//                 obstacle.setStartPoint(j, i);
//                 start = State(j, i);
//             }
//             else if (c == '*')
//             {
//                 obstacle.setGoalPoint(j, i);
//                 goal = State(j, i);
//             }
//             else
//             {

//                 obstacle.addDynamicObstacle(10, 10, 1, c, j, i);
//             }
//         }
//     }
// }

// int main(int argc, char *argv[])
// {
//     int lookahead[7] = {1, 3, 10, 30, 100, 300, 1000};
//     //int lookahead[9] = {150,160,170,180,190,155,165,175,185};
//     setObstacles(argc, argv);

//     vector<DynamicObstacle> dobs = obstacle.getDynamicObstacle1();
//     for (int k = Movement; k <= Movement; k *= 10)
//     {
//         cerr << "MOVEMENT " << k << endl;
//         cout << "movements " << k << endl;
//         for (int i = 0; i < 12; i++) //for (int i = 1; i < 4; i+=2)
//         {
//             cout << "algorithm " << i << endl;
//             cerr << "algorithm " << i << endl;
//             obstacle.setMode(i);
//             for (int j = 0; j < 7; j++)
//             {
//                 obstacle.setDynamicObstacle(dobs);
//                 obstacle.setStartPoint(start.x, start.y);
//                 obstacle.setGoalPoint(goal.x, goal.y);
//                 cout << lookahead[j] << " " << obstacle.MoveObstacle(k, lookahead[j]) << endl;
//             }
//         }
//     }

//     // int lookahead[5] = {10,30,100,300,1000};
//     // int lookahead[5] = {1,3,5,7,10};
//     // setObstacles(argc, argv);

//     // vector<DynamicObstacle> dobs = obstacle.getDynamicObstacle1();
//     // for (int k = 1000; k <= 1000; k *= 10)
//     // {
//     //     cerr << "MOVEMENT " << k << endl;
//     //     cout << "movements " << k << endl;
//     //     for (int i = 0; i < 2; i++) //for (int i = 1; i < 4; i+=2)
//     //     {
//     //         int alg = i % 2 ? 1 : 4;
//     //         cout << "algorithm " << alg << endl;
//     //         cerr << "algorithm " << alg << endl;
//     //         obstacle.setMode(alg);
//     //         for (int j = 1; j < 11; j++)
//     //         {
//     //         obstacle.setDynamicObstacle(dobs);
//     //         obstacle.setStartPoint(start.x, start.y);
//     //         obstacle.setGoalPoint(goal.x, goal.y);
//     //         // cout << lookahead[j] << " " << obstacle.MoveObstacle(k, lookahead[j]) << endl;
//     //         cout << j << " " << obstacle.MoveObstacle(k, j) << endl;
//     //         }
//     //     }
//     // }
// }
