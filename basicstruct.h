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
using namespace std;
//可能没用,向量的类定义，vector可能有歧义，这里使用了向量的另一个名字哈哈哈哈

struct directed_quantity{
    double x,y;
   directed_quantity(){
        x=0;y=0;
    }
   directed_quantity(double a,double b){
        x=a;
        y=b;
    }
    //向量运算
    friend directed_quantity operator-(directed_quantity a,directed_quantity b){
        return directed_quantity(a.x-b.x,a.y-b.y);
    }
    friend directed_quantity operator+(directed_quantity a,directed_quantity b){
        return directed_quantity(a.x+b.x,a.y+b.y);
    }
};
//定义点
struct Point{
    double x,y;
    //计算两个点之间的距离
    friend double distance(Point a,Point b){
        return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    }
    //两个点形成向量
    friend directed_quantity operator -(Point a,Point b){
        return directed_quantity(a.x-b.x,a.y-b.x);
    }
};
//定义路径
struct Edge{
    int P1,P2;
    double length,reallength;//路的长度，时间，考虑后续名字改一下
    unsigned capacity,flow;//车容量，流量
    Edge(){flow=length=0;}
    Edge(int a,int b,double dis,int cap,int f=0):P1(a),P2(b),length(dis),capacity(cap),flow(f){}
};
class Graph{};
