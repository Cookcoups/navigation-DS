#ifndef BASICSTRUCT_H
#define BASICSTRUCT_H
#endif // BASICSTRUCT_H
#include <iostream>
#include "qnamespace.h"
#include <windows.h>
#include <cmath>
#include<climits>
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
    double length,realtime;//路的长度，时间，考虑后续名字改一下
    unsigned capacity,flow;//车容量，流量
    Edge(){flow=length=0;}
    Edge(int a,int b,double dis,int cap,int f=0):P1(a),P2(b),length(dis),capacity(cap),flow(f){}
};
class Graph{
    bool Randflg;
    unsigned int PointNums;
public:vector<vector<pair<int,int>>>G;//first记录其目的点标号, second记录该边标号
    vector<Point>P;//存放点的信息，标号从1开始
    vector<Edge>E;//存放边的信息，标号从0开始
private:
    vector<bool>visited;
    void dfs(int now,int father)
    {
        visited[now]=true;
        cout<<now<<' '<<father<<endl;
        for(auto [to,edge]:G[now])
        {
            if(visited[to])
                continue;
            //printf("%d ---> %d : %d/%d (%lf)\n", n, to, E[e].flow, E[e].capacity, 1.0 * E[e].flow / E[e].capacity);
            cout << now << " ---> " << to << " : " << E[edge].flow << "/" << E[edge].capacity << " (" << (1.0 * E[edge].flow / E[edge].capacity) << ")" << endl;
            dfs(to,now);
        }
    }
public:Graph(int n)
    {

    }
    ~Graph(){}
    // 返回输入点附近的100个点的标号
    vector<int> get100points(Point p)
    {
        vector<int>ret;
        priority_queue<pair<double,int>>pq;
        for(int i=1;i<=(int)PointNums;i++)
        {
            pq.push({distance(P[i],p),i});
            if(pq.size()>100)pq.pop();
        }
        // 把最近的至多100个点放入ret返回.
        while(!pq.empty()){
            auto x = pq.top();
            pq.pop();
            ret.push_back(x.second);
        }
        return ret;
    }
    // 返回邻接链表的头指针
    vector<vector<pair<int, int>>>::iterator GraphHead()
    {
        return G.begin();
    }
    //返回点的数目
    unsigned GetPointNums()
    {
        return PointNums;
    }
    //返回边的数目
    unsigned GetEdgeNums()
    {
        return (unsigned)E.size();
    }
    // 返回点池的首指针
    vector<Point>::iterator PointHead()
    {
        return P.begin();
    }
    // 返回边池的首指针
    vector<Edge>::iterator EdgeHead()
    {
        return E.begin();
    }
    void Time(vector<Edge>::iterator E,Graph *G)
    {
        for(int i=0;i<(int)G->GetPointNums();i++)
        {
            if(E[i].flow<=E[i].capacity)
                E[i].realtime=E[i].length/50;
            else {
                double x=(double)E[i].flow/E[i].capacity;
                E[i].realtime=E[i].length/50*(1+exp(x));
            }
        }
    }
    //返回最短路径
    vector<int>dij(int start,int end,Graph *Gp,bool choice,double &distance,double & time)
    {
        time=0;
        distance =0;
        vector<int>path;
        auto E=Gp->EdgeHead();
        int n=Gp->GetPointNums();
        auto G=Gp->GraphHead();
        Time(E,Gp);
        vector<double>dis(n+1,INT_MAX);
        priority_queue<pair<double,pair<int,int>>>pq;
        pq.push({0,{start,0}});
        vector<bool>visited(n+1);
        vector<int>father(n+1);
        while(!pq.empty())
        {
            auto [len,cur]=pq.top();
            auto [now,fa]=cur;
            pq.pop();
            if(visited[now])
                continue;
            father[now]=fa;
            visited[now]=true;
            dis[now]=-len;
            for(auto [to,edge]:G[now])
            {
                if(visited[to])
                    continue;
                double l=0;
                if(choice)
                    l=len-E[edge].realtime;
                else l=len-E[edge].length;
                pq.push({l,{to,edge}});
            }
            if(now==end)
            {
                while(now!=start)
                {
                    path.push_back(father[now]);
                    now=E[father[now]].P1;
                }
                for(int i=0;i<(int)path.size();i++)
                {
                    time+=E[path[i]].realtime;
                    distance+=E[path[i]].length;
                }
                return path;
            }
        }
        return path;
    }
};
