#include <gtest/gtest.h>
#include <iostream>
#include <memory>

template <class Data>
struct Iterator 
{
public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = Data;
    using pointer           = Data*;
    using reference         = Data&;
    Iterator(pointer ptr) : m_ptr(ptr) {}
    reference operator*() const { return *m_ptr; }
    pointer operator->() { return m_ptr; }
    Iterator& operator++() { m_ptr++; return *this; } 
    Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }
    friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_ptr == b.m_ptr; };
    friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_ptr != b.m_ptr; };     
protected:
    pointer m_ptr;
};



class A{
public:
    A(){std::cout << "Type A\n";};
    ~A(){};

    class AIN{
    public:
        AIN(){std::cout << "Type AIN\n";};
        ~AIN(){};
        virtual void print() = 0;
    };
    void print(){std::cout << "I'm A\n";};

    auto getDefaultInt(){return 3;};
};

class B{
public:
    B(const std::array<std::shared_ptr<int>,3>& data) : data_(data){
        
        std::cout << "Type B\n";};
    ~B(){};

    using intB =  int;

    Iterator<const std::shared_ptr<int>> begin(){return Iterator<const std::shared_ptr<int>>(&data_[0]);}
    Iterator<const std::shared_ptr<int>> end(){return Iterator<const std::shared_ptr<int>> (&data_[3]);}
    
    void print(){std::cout << "I'm B\n";}
    void print2(){std::cout << "I'm B2\n";}

    const std::array<std::shared_ptr<int>,3> data_;
};

// class C: public A,B{
// public:
//     C(){std::cout << "Type C\n";};
//     ~C(){};
//     void print(){std::cout << "I'm C\n";}
// };

class D: public A{
public:
    D(){std::cout << "Type D\n";};
    ~D(){};

    class DIN:AIN{
    public:
        DIN(){std::cout << "Type DIN\n";};
        ~DIN(){};
        virtual void print()override {std::cout << "I'm DIN\n";}

    };

    void print(){std::cout << "I'm D\n";}
};


template<class T> 
class Temp {
public:
    Temp(){std::cout << "Type Temp\n";};//auto leg_data_map = hyq.legDataMap<int>();
    
    ~Temp(){};
    void print(){std::cout << "I'm Temp\n";}
};

template<template<typename> class T> 
class TempTemp {
public:
    TempTemp(){std::cout << "Type TempTemp\n";};
    ~TempTemp(){};
    void print(){std::cout << "I'm TempTemp\n";}
};


#include <iostream>
#include <string>
using namespace std;

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    // A* c = new C();
    // c->print();
    
    // std::shared_ptr<A> a = std::make_shared<D>();
    
    // auto leg_data_map = hyq.legDataMap<int>();
    // A a;

    // std::cout << "AUTO " << a.getDefaultInt() <<std::endl;

    // Temp<int> t;
    // TempTemp<Temp> tt;    
    // D::AIN ain;
    // ain.print();
    std::array<std::shared_ptr<int>,3> data {
    make_shared<int>(3),
    make_shared<int>(4),
    make_shared<int>(5),
    };
    
    // B b(data);
    // int i =0;

    // for(auto x:b){
    //     *x = ++i;

    //     std:: cout << *x << std::endl;

    // }

    return RUN_ALL_TESTS();
}