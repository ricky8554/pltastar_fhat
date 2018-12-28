#include "HashSet.h"
#include <cassert>
#include <iostream>
#include <functional>
#include <unordered_set>
#include <ctime>
using namespace std;

unsigned int compare(int *const &s1, int *const &s2)
{
    return *s1 == *s2;
};

unsigned long hashf(int *const &s1)
{
    hash<int> ptr_hash;
    return ptr_hash(*s1);
};

int main()
{

    // clock_t begin = clock();
    // unordered_set<int> s2;
    // for (int i = 0; i < 10000000; i++)
    // {
    //     s2.insert(i);
    // }
    // //assert(s1.size()==100);
    // cout << s2.size() << endl;
    // s2.insert(14);
    // //assert(s1.size()==100);
    // cout << s2.size() << endl;
    // for (int i = 0; i < 10000000; i++)
    // {
    //     assert(s2.find(i) != s2.end());
    // }
    // assert(s2.find(100000000) == s2.end());

    // clock_t end = clock();

    // double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // cout << elapsed_secs << endl;

    // begin = clock();

    unordered_set<int *> s2;

    HashSet<int *> s1(&hashf, &compare);
    for (int i = 0; i < 100000; i++)
    {
        s1.insert(new int(i));
    }
    //assert(s1.size()==100);
    s1.erase(new int(14));
    //assert(s1.size()==100);
    cout << s1.size() << endl;
    // for (int i = 0; i < 10000000; i++)
    // {
    //     assert(s1.find(i));
    // }
    assert(!s1.find(new int(14)));
    assert(s1[new int(14)] == NULL);

    cout << s1[new int(14)] << endl;

    cout << *s1[new int(15)] << endl;
    int count = 0;
    HashSet<int *> s3(s1);
    for(auto i:s3)
    {   
        count ++;
       // cout << "test " << *(i.key)<<endl;
    }
    cout << s1.find(new int(44)) << endl;



    // end = clock();

    // elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // cout << elapsed_secs << endl;
}
