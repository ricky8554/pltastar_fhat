#include "PQueue.h"
#include <cstdlib>
#include <cassert>
#include <queue>
#include <iostream>
using namespace std;

unsigned int compare(int &t1, int &t2)
{
    return t1 < t2;
}

unsigned int compare1(int &t1, int &t2)
{
    return t1 > t2;
}

int main(){

    PQueue<int> test(&compare1);
    priority_queue<int> test1;
    srand(time(NULL));


    

    for(int i = 99999999; i>= 0;i--)
    {
        test1.push(rand());
    }

    clock_t begin1 = clock();

    for(int i = 99999999; i>= 0;i--)
    {
        
        test1.pop();
    }

    clock_t end1 = clock();
    double elapsed_secs1 = double(end1 - begin1) / CLOCKS_PER_SEC;


    

        //test.pop();
    //test.remove();
    

    for(int i = 99999999; i>= 0;i--)
    {
        test.push(rand());
    }
    clock_t begin = clock();
     

    for(int i = 99999999; i>= 0;i--)
    {
        test.remove();
    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;


   

    cout << "MINE: " <<elapsed_secs << " STL: " << elapsed_secs1 << endl;
    /*
    for(int i = 0; i< 99; i++)
    {
        test.push(i);
    }
    for(int i = 0; i< 99; i++)
    {
        cout << test.top() << endl;
        test.remove();
    }
    for(int i = 0; i< 9999; i++)
    {
        test.push(i);
    }
    for(int i = 0; i< 9999; i++)
    {
        cout << test.pop() << endl;
    }
   

   

    for(int i = 0; i< 9999; i++)
    {
        test.push(i);
    }

    test.setUpCompare(&compare1);

    for(int i = 0; i< 9999; i++)
    {
        cout << test.pop() << endl;
    }
     */
     

    /*for(int i = 0; i< 9999; i++)
    {
        test.push(i);
    }

    for(int i = 1000; i < 2000;i++)
    {
        test[i] = 0;
    }
    test.reBuild();

    for(int i = 0; i< 9999; i++)
    {
        cout << test.pop() << endl;
    }*/

}

