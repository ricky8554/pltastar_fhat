//========================================================================
// modify from cs770
// modify: Chao Chi Cheng
// Date: Aug. 2018
//========================================================================

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include <string>

#include "../utility/camera.h"
#include "executive.h"

#define PIR 3.14159265 / 180

using namespace std;

Camera cam;
Executive obstacle;
vector<DynamicObstacle> dynamicObstacles;
unordered_set<StaticObstacle> staticObstacles;
GLuint list;
State goal;
State start;
int boardw = 0, boardh = 0;
// double maxdouble = -1;

//////////////////////////////////////////////////////////////////////
//
// Call this to redraw the scene.
//
//////////////////////////////////////////////////////////////////////

void getStaticObstacle()
{
    staticObstacles = obstacle.getStaticObstacle();
    list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glColor3d(0, 0, 0);
    for (StaticObstacle staticobs : staticObstacles)
    {
        glBegin(GL_POLYGON);
        glVertex2d(staticobs.x, staticobs.y);
        glVertex2d(staticobs.x + 1, staticobs.y);
        glVertex2d(staticobs.x + 1, staticobs.y + 1);
        glVertex2d(staticobs.x, staticobs.y + 1);
        glEnd();
    }
    glEndList();
}

void getDynamicObstacle()
{
    dynamicObstacles = obstacle.getDynamicObstacle();
}
int getindex(int x, int y)
{
    return boardw * y + x;
}

void display(void)
{

    glClearColor(.9f, .9f, .9f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    cam.begin_drawing();

    unordered_set<State> state = obstacle.getSTATE();

    vector<State> path = obstacle.getPath();

    // for (int i = 0; i < 100; i++)
    // {
    //     for (int j = 0; j < 100; j++)
    //     {
    //         State s(i, j);
    //         if (state.find(s) == state.end())
    //         {
    //             float x = i, y = j, h = s - goal;
    //             glColor4f(1, 0, 0, 1 * h / 126);
    //             glBegin(GL_POLYGON);
    //             glVertex2d(x + 0.5, y);
    //             glVertex2d(x - 0.5, y);
    //             glVertex2d(x - 0.5, y + 1);
    //             glVertex2d(x + 0.5, y + 1);
    //             glEnd();
    //         }
    //     }
    // }

    int arr[boardw * boardh];

    for (int i = 0; i < boardw * boardh; i++)
        arr[i] = 0;

    for (State s : path)
    {
        ++arr[getindex(s.x, s.y)];
    }

    for (int i = 0; i < boardw * boardh; i++)
    {

        glColor4d(0, 0.6, 0.6, 0.2 * arr[i]); // * (elem.f() - minstate + 1) / (maxstate -minstate + 1));
        float y = i / boardw, x = i % boardw;
        glBegin(GL_POLYGON);
        glVertex2d(x, y);
        glVertex2d(x + 1, y);
        glVertex2d(x + 1, y + 1);
        glVertex2d(x, y + 1);
        glEnd();
    }

    double maxstate = 0;
    double minstate = 1000000;
    for (const auto &elem : state)
    {
        if (elem.f() < 1000)
        {
            if (elem.f() > maxstate)
                maxstate = elem.f();

            if (elem.f() < minstate)
                minstate = elem.f();
        }
    }

    for (const auto &elem : state)
    {
        if ((elem.f()) < 1000)
        {
            glColor4d(1, 0, 1, 1 * (elem.f() - minstate + 1) / (maxstate - minstate + 1));
            float x = elem.x, y = elem.y;
            glBegin(GL_POLYGON);
            glVertex2d(x, y);
            glVertex2d(x + 1, y);
            glVertex2d(x + 1, y + 1);
            glVertex2d(x, y + 1);
            glEnd();
        }
    }
    //call dynamic obstacle
    glCallList(list);

    glColor3d(1, 0, 0);

    glBegin(GL_TRIANGLE_FAN);

    double x = goal.x + 0.5, y = goal.y + 0.5, radius = 0.5;
    glColor3d(0, 1, 0);
    glVertex2f(x, y);
    for (int i = 0; i <= 360; i++)
        glVertex2f(radius * cos(PIR * i) + x, radius * sin(PIR * i) + y);
    glEnd();

    x = start.x + 0.5;
    y = start.y + 0.5;

    radius = 0.5;
    start = obstacle.getStatePoint();
    glBegin(GL_TRIANGLE_FAN);
    glColor3d(0, 0, 1);
    glVertex2f(x + 0.5, y + 0.5);
    for (int i = 0; i <= 360; i++)
        glVertex2f(radius * cos(PIR * i) + x, radius * sin(PIR * i) + y);
    glEnd();

    getDynamicObstacle();
    //glBegin(GL_POINTS);
    glColor3d(1, 0, 0);
    for (DynamicObstacle &dynaimcobs : dynamicObstacles)
    {
        glBegin(GL_TRIANGLE_FAN);
        double x = dynaimcobs.x + 0.5, y = dynaimcobs.y + 0.5, radius = 0.5;
        glVertex2f(x, y);
        for (int i = 0; i <= 360; i++)
            glVertex2f(radius * cos(PIR * i) + x, radius * sin(PIR * i) + y);
        glEnd();
        double dhead = dynaimcobs.estimate_h, dspeed = dynaimcobs.estimate_s;
        for (int i = 1; i < 6; i++)
        {
            double dyx = cos(dhead) * dspeed * i + dynaimcobs.x + 0.5, dyy = sin(dhead) * dspeed * i + dynaimcobs.y + 0.5;
            // double powt = pow(1.3, i); //change 1.2 to any
            double var = (1 + 0.1 * i) * (1 + 0.1 * i);
            double radius = sqrt(var * var);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i <= 360; i++)
                glVertex2f(radius * cos(PIR * i) + dyx, radius * sin(PIR * i) + dyy);
            glEnd();
        }
    }
}

static void error_callback(int error, const char *description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow *window, int key,
                         int scancode, int action, int mods)
{
    if ((key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) &&
        action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    if (key == GLFW_KEY_K)
        obstacle.terminate();
}

void setObstacles(int argc, char *argv[])
{
    obstacle.setMode(0);
    if (argc > 1)
    {
    }
    else
        obstacle.setMode(0);

    bool map = false;
    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-map"))
        {
            map = true;
        }
        else if(!strcmp(argv[i], "-a"))
        {
            if (!strcmp(argv[i+1], "LSS_LRTA") || !strcmp(argv[i+1], "0"))
                obstacle.setMode(0);
            else if(!strcmp(argv[i+1], "-1"))
                obstacle.setMode(-1);
            else if (!strcmp(argv[i+1], "PLRTA_STAR") || !strcmp(argv[i+1], "1"))
                obstacle.setMode(1);
            else if (!strcmp(argv[i+1], "LSS_LRTA_FHAT") || !strcmp(argv[i+1], "2"))
                obstacle.setMode(2);
            else if (!strcmp(argv[i+1], "PLRTA_STAR_FHAT") || !strcmp(argv[i+1], "3"))
                obstacle.setMode(3);
            else if (!strcmp(argv[i+1], "PLRTA_STAR_MOD") || !strcmp(argv[i+1], "4"))
                obstacle.setMode(4);
            else if (!strcmp(argv[i+1], "PLRTA_STAR_FHAT_MOD") || !strcmp(argv[i+1], "5"))
                obstacle.setMode(5);
            else if (!strcmp(argv[i+1], "DYNAMIC_LSS_LRTA") || !strcmp(argv[i+1], "6"))
                obstacle.setMode(6);
            else if (!strcmp(argv[i+1], "DYNAMIC_PLRTA_STAR") || !strcmp(argv[i+1], "7"))
                obstacle.setMode(7);
            else if (!strcmp(argv[i+1], "DYNAMIC_LSS_LRTA_FHAT") || !strcmp(argv[i+1], "8"))
                obstacle.setMode(8);
            else if (!strcmp(argv[i+1], "DYNAMIC_PLRTA_STAR_FHAT") || !strcmp(argv[i+1], "9"))
                obstacle.setMode(9);
            else if (!strcmp(argv[i+1], "DYNAMIC_PLRTA_STAR_MOD") || !strcmp(argv[i+1], "10"))
                obstacle.setMode(10);
            else if (!strcmp(argv[i+1], "DYNAMIC_PLRTA_STAR_FHAT_MOD") || !strcmp(argv[i+1], "11"))
                obstacle.setMode(11);
            else if (!strcmp(argv[i+1], "12"))
                obstacle.setMode(12);
            else if (!strcmp(argv[i+1], "13"))
                obstacle.setMode(13);
            else
                obstacle.setMode(0);
        }
        else if(!strcmp(argv[i], "-l"))
        {
            obstacle.setLookAhead(stoi(argv[i+1]));
        }
        else if(!strcmp(argv[i], "-r"))
        {
            obstacle.setRate(stoi(argv[i+1]));
        }
        else if(!strcmp(argv[i], "-help"))
        {
            cerr << "<-r int> updaterate\n<-l int> lookahead size\n<-a int/string> algorithm\n<-help> for help" << endl;
            exit(-1);
        }
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
                    // obstacle.addDynamicObstacle(10, 10, 1,c, j, i);
                }
                else if (c == 'S')
                {
                    obstacle.setStartPoint(j, i);
                    start = State(j, i);
                }
                else if (c == 'G')
                {
                    obstacle.setGoalPoint(j, i);
                    goal = State(j, i);
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
                    start = State(j, i);
                }
                else if (c == '*')
                {
                    obstacle.setGoalPoint(j, i);
                    goal = State(j, i);
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    GLFWwindow *window;
    setObstacles(argc, argv);

    int w = 800;
    int h = 800;
    if (boardw > boardh)
    {
        w = 800;
        h = w * boardh / boardw;
    }
    else
    {
        h = 800;
        w = h * boardw / boardh;
    }

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        cerr << "glfwInit failed!\n";
        cerr << "PRESS Control-C to quit\n";
        char line[100];
        cin >> line;
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(w, h, "C Cheng", NULL, NULL);

    if (!window)
    {
        cerr << "glfwCreateWindow failed!\n";
        cerr << "PRESS Control-C to quit\n";
        char line[100];
        cin >> line;

        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    cam = Camera(0, 0, boardw, boardh, w, h, window);

    glfwSetKeyCallback(window, key_callback);

    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(1);
    //new
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //START OF OBSTACLE

    getStaticObstacle();
    getDynamicObstacle();
    obstacle.scheduledChanged();

    //END OF OBSTACLE

    while (!glfwWindowShouldClose(window))
    {
        cam.check_resize();

        display();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    //terminate updating
    obstacle.terminate();

    glfwDestroyWindow(window);

    glfwTerminate();
    exit(EXIT_SUCCESS);
}

//! [code]