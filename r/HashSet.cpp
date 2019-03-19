
template <typename T>
HashSet<T>::HashSet(size_t (*h)(const T &t), T defaltT)
{
    hash = h;
    empty = defaltT;
    _size = 0;
    tablesize = nextPrime(tablesize);
    array = new Elem *[tablesize]();
}

template <typename T>
HashSet<T>::HashSet(size_t (*h)(const T &t), unsigned int (*c)(const T &t1, const T &t2), T defaltT)
{
    hash = h;
    empty = defaltT;
    compare = c;
    _size = 0;
    tablesize = nextPrime(tablesize);
    array = new Elem *[tablesize]();
}

// copy constructor
template <typename T>
HashSet<T>::HashSet(const HashSet &rhs)
{
    tablesize = rhs.tablesize;
    compare = rhs.compare;
    hash = rhs.hash;
    _size = 0;

    array = new Elem *[tablesize]();

    for (int i = 0; i < tablesize; ++i)
    {
        Elem *tempentry = rhs.array[i];
        while (tempentry)
        {
            insert(tempentry->key);
            tempentry = tempentry->next;
        }
    }
}

// destructor
template <typename T>
HashSet<T>::~HashSet()
{
    for (int i = 0; i < tablesize; ++i)
    {
        Elem *tempentry = array[i], *temp;
        while (tempentry)
        {
            temp = tempentry;
            tempentry = tempentry->next;
            delete temp;
        }
    }
    delete[] array;
}

template <typename T>
void HashSet<T>::clear()
{
    for (int i = 0; i < tablesize; ++i)
    {
        Elem *tempentry = array[i], *temp;
        while (tempentry)
        {
            temp = tempentry;
            tempentry = tempentry->next;
            delete temp;
        }
        array[i] = 0;
    }
    _size = 0;
}

// assignment operator
template <typename T>
HashSet<T> &HashSet<T>::operator=(const HashSet &rhs)
{
    if (this == &rhs)
        return *this;
    for (int i = 0; i < tablesize; ++i)
    {
        Elem *tempentry = array[i];
        while (tempentry)
        {
            Elem *a = tempentry;
            tempentry = tempentry->next;
            delete a;
        }
    }
    delete[] array;

    tablesize = rhs.tablesize;
    compare = rhs.compare;
    hash = rhs.hash;
    _size = 0;

    array = new Elem *[tablesize]();

    for (int i = 0; i < tablesize; ++i)
    {
        Elem *tempentry = rhs.array[i];
        while (tempentry)
        {
            insert(tempentry->key);
            tempentry = tempentry->next;
        }
    }

    return *this;
}

template <typename T>
void HashSet<T>::setUpCompare(unsigned int (*c)(const T &t1, const T &t2))
{
    compare = c;
}

// insert an element into the set;
template <typename T>
bool HashSet<T>::insert(T data)
{
    if (_size == tablesize)
        rehash();

    unsigned int keyindex = hash(data) % tablesize;

    Elem *temp = array[keyindex];

    if (!temp)
    {
        array[keyindex] = new Elem(data, 0);
        ++_size;
    }
    else
    {
        if (compare(temp->key, data))
        {
            return false;
        }
        while (temp->next)
        {
            if (compare(temp->key, data))
                return false;
            temp = temp->next;
        }
        temp->next = new Elem(data, 0);
        ++_size;
    }
    return true;
}

// remove an element from the set; not implemented
template <typename T>
bool HashSet<T>::erase(T data)
{
    if (_size == 0)
        return false;

    unsigned int keyindex = hash(data) % tablesize;

    Elem *temp = array[keyindex];

    if (temp)
    {
        if (compare(temp->key, data))
        {
            array[keyindex] = temp->next;
            delete temp;
            --_size;
            return true;
        }
        while (temp->next)
        {
            if (compare(temp->next->key, data))
            {
                Elem *a = temp->next;
                temp->next = a->next;
                delete a;
                --_size;
                return true;
            }
            temp = temp->next;
        }
    }
    return false;
}
template <typename T>
void HashSet<T>::rehash()
{
    int temptablesize = tablesize;
    tablesize = nextPrime(tablesize << 1);
    Elem **array1 = array;
    _size = 0;
    array = new Elem *[tablesize]();

    for (int i = 0; i < temptablesize; ++i)
    {
        Elem *tempentry = array1[i];
        while (tempentry)
        {
            Elem *a = tempentry;
            tempentry = tempentry->next;
            insert(a);
        }
    }
    delete[] array1;
}

// If found, return true
template <typename T>
bool HashSet<T>::find(T data) const
{
    if (_size == 0)
        return false;

    unsigned int keyindex = hash(data) % tablesize;

    Elem *temp = array[keyindex];
    while (temp)
    {
        if (compare(temp->key, data))
            return true;
        temp = temp->next;
    }
    return false;
}

// return size of the set
template <typename T>
unsigned int HashSet<T>::size() const
{
    return _size;
}

template <typename T>
unsigned int HashSet<T>::tableSize() const
{
    return tablesize;
}

template <typename T>
void HashSet<T>::printBucket()
{
    for (int i = 0; i < tablesize; ++i)
    {
        Elem *tempentry = array[i];
        int count = 0;
        while (tempentry)
        {
            ++count;
            tempentry = tempentry->next;
        }
        cout << "Bucket " << i << ": " << count << endl;
    }
}

template <typename T>
void HashSet<T>::destructCode(Elem *&)
{
}

// helper method for inserting a node
template <typename T>
void HashSet<T>::insert(Elem *&elem)
{
    T data = elem->key;
    elem->next = array[hash(data) % tablesize];
    array[hash(data) % tablesize] = elem;
    ++_size;
}

// Internal method to test if a positive number is prime (not efficient)
template <typename T>
bool HashSet<T>::isPrime(unsigned int number)
{
    if (number % 3 == 0)
        return false;

    int divisor = 6;
    while (divisor * divisor - 2 * divisor + 1 <= number)
    {

        if (number % (divisor - 1) == 0)
            return false;

        if (number % (divisor + 1) == 0)
            return false;

        divisor += 6;
    }
    return true;
}

// Internal method to return a prime number at least as large as n.
// Assumes n > 0.
template <typename T>
unsigned int HashSet<T>::nextPrime(unsigned int n)
{
    if (n % 2 == 0)
        n++;
    for (; !isPrime(n); n += 2)
        ;
    return n;
}

template <typename T>
typename HashSet<T>::Iterator HashSet<T>::findIt(const T &data) const
{

    unsigned int keyindex = hash(data) % tablesize;
    Elem *temp = array[keyindex];
    while (temp)
    {
        if (compare(temp->key, data))
            return Iterator(temp, this);
        temp = temp->next;
    }

    return end();
}

template <typename T>
T HashSet<T>::operator[](const T &data)
{

    //empty T
    unsigned int keyindex = hash(data) % tablesize;
    Elem *temp = array[keyindex];
    while (temp)
    {
        if (compare(temp->key, data))
            return temp->key;
        temp = temp->next;
    }
    return 0;
}

template <typename T>
typename HashSet<T>::Iterator HashSet<T>::Iterator::operator++()
{
    // stub code; need to implement !!!
    if (_cur == NULL)
        return Iterator(NULL, _map);

    Iterator result = *this;
    if (_cur->next != NULL)
    {
        _cur = _cur->next;
    }
    else
    {
        unsigned int keyindex = _map->hash(_cur->key) % _map->tablesize;

        for (int i = keyindex + 1; i < _map->tablesize; i++)
        {
            if (_map->array[i] != NULL)
            {
                _cur = _map->array[i];
                return result;
            }
        }
        _cur = NULL;
    }

    return result;
}

template <typename T>
typename HashSet<T>::Iterator HashSet<T>::Iterator::operator++(int)
{
    Iterator result = *this;
    return ++result;
}

template <typename T>
typename HashSet<T>::Iterator HashSet<T>::begin() const
{
    // stub code; need to implement !!!
    for (int i = 0; i < tablesize; i++)
    {
        if (array[i] != 0)
            return Iterator(array[i], this);
    }
    return end();
}

template <typename T>
typename HashSet<T>::Iterator HashSet<T>::end() const
{
    // stub code; need to implement !!!
    return Iterator(NULL, this);
}

template <typename T>
typename HashSet<T>::Elem *HashSet<T>::Iterator::operator->()
{
    return _cur;
}

template <typename T>
typename HashSet<T>::Elem &HashSet<T>::Iterator::operator*()
{
    return *_cur;
}

template <typename T>
bool HashSet<T>::Iterator::operator!=(Iterator it)
{
    return _cur != it._cur;
}

template <typename T>
bool HashSet<T>::Iterator::operator==(Iterator it)
{
    return _cur == it._cur;
}