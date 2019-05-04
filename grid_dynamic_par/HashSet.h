// a partial implementation of set ADT with an AVL tree

#ifndef _HASH_SET_H_
#define _HASH_SET_H_

#include <iostream>
#include <string>
#include <algorithm> // for max()
using namespace std;

template <typename T>
class HashSet
{

private:
  struct Elem
  {
    T key;
    Elem *next;
    Elem(const T &key, Elem *next)
        : key(key), next(next) {}
    Elem(const T &key)
        : key(key) {}
    Elem() {}
  };

  Elem **array;                  // pointers to the hash table, each table entry is a pointer that used to point to a linked list of Entries.
  unsigned int tablesize = 2000; // table size
  unsigned int _size;
  size_t (*hash)(const T &t);
  unsigned int (*compare)(const T &t1, const T &t2);
  T empty;

  // common code for deallocation
  void destructCode(Elem *&);

  void rehash();

  // helper method for inserting a node
  void insert(Elem *&);

  // check if the number n is prime
  bool isPrime(unsigned int n);

  // find the next prime number following given n
  unsigned int nextPrime(unsigned int n);

public:
  // default constructor; constructs empty set
  HashSet(size_t (*hash)(const T &t), T defaltT);

  HashSet(size_t (*hash)(const T &t), unsigned int (*compare)(const T &t1, const T &t2) , T defaltT);

  // copy constructor
  HashSet(const HashSet &);

  // destructor
  ~HashSet();

  // assignment operator
  HashSet &operator=(const HashSet &);

  void clear();

  // insert an element into the set;
  bool insert(T);

  // remove an element from the set; not implemented
  bool erase(T);

  // If found, return true
  bool find(T) const;

  // return size of the set
  unsigned int size() const;

  unsigned int tableSize() const;

  void printBucket();

  void setUpCompare(unsigned int (*c)(const T &t1, const T &t2));

  class Iterator
  {
  public:
    Iterator() {}
    explicit Iterator(Elem *cur, const HashSet *map) : _cur(cur), _map(map) {}
    Elem &operator*();
    Elem *operator->();
    Iterator operator++();
    Iterator operator++(int);
    bool operator!=(Iterator);
    bool operator==(Iterator);

  private:
    Elem *_cur;
    const HashSet *_map;
  };

  Iterator begin() const;

  Iterator end() const;

  Iterator findIt(const T &data) const;

  T operator[](const T &data);
};

#include "HashSet.cpp"
#endif
