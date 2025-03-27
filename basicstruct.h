#ifndef BASICSTRUCT_H
#define BASICSTRUCT_H

#endif // BASICSTRUCT_H

#include <iostream>
#include "qnamespace.h"
#include <windows.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <algorithm>
#include <map>
#include <queue>
#include <random>
//实现了向量，点，边的struct定义
//初始创建了graph类
using namespace std;
//可能没用
struct Vector{
    double x,y;
    Vector(){
        x=0;y=0;
    }
    Vector(double a,double b){
        x=a;
        y=b;
    }
    //向量运算
    friend Vector operator-(Vector a,Vector b){
        return Vector(a.x-b.x,a.y-b.y);
    }
    friend Vector operator+(Vector a,Vector b){
        return Vector(a.x+b.x,a.y+b.y);
    }
};
struct Point{
    double x,y;
    friend double distance(Point a,Point b){
        return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));//两点间的距离
    }
    //两个点形成向量
    friend Vector operator -(Point a,Point b){
        return Vector(a.x-b.x,a.y-b.x);
    }
};
struct Edge{
    int P1,P2;
    double length,reallength;
    unsigned capacity,flow;
    Edge(){flow=length=0;}
    Edge(int a,int b,double dis,int cap,int f=0):P1(a),P2(b),length(dis),capacity(cap),flow(f){}
};
class Graph{};
