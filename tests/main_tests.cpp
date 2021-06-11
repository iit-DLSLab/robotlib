#include <gtest/gtest.h>
#include <iostream>
#include <memory>

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
    void print(){std::cout << "I'm A\n";}
};
class B{
public:
    B(){std::cout << "Type B\n";};
    ~B(){};
    void print(){std::cout << "I'm B\n";}
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
    Temp(){std::cout << "Type Temp\n";};
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


//: C10:InnerClassIdiom.cpp
// Example of the "inner class" idiom.
#include <iostream>
#include <string>
using namespace std;


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // A* c = new C();
    // c->print();
    
    std::shared_ptr<A> a = std::make_shared<D>();
    
    //Temp<int> t;
    //TempTemp<Temp> tt;
    
    //D::AIN ain;

    //ain.print();


    return RUN_ALL_TESTS();
}