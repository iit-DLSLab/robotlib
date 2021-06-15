#include <iostream>
#include <memory>
#include "utils.hpp"

using namespace std;
using namespace dls::robot;

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



int main(){

    std::array<std::shared_ptr<int>,3> data {
    make_shared<int>(3),
    make_shared<int>(4),
    make_shared<int>(5),
    };

    return 0;
}