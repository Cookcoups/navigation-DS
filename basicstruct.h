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
//可能没用,向量的类定义，vector可能有歧义，这里使用了向量的另一个名字111

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
//定义随机数函数，生成的随机数为0到range-1
unsigned long long myrand(unsigned long long range = 0) {
    // 每个线程拥有独立的随机数生成器实例，避免多线程竞争
    thread_local std::mt19937_64 engine = []() {
        // 使用真随机数设备生成种子序列
        std::random_device rd;
        std::vector<unsigned int> seed_data(4); // 使用4个随机数作为种子
        for (auto& elem : seed_data) {
            elem = rd();
        }
        std::seed_seq seq(seed_data.begin(), seed_data.end());
        return std::mt19937_64(seq);
    }();

    if (range == 0) {
        // 直接返回64位随机数
        return engine();
    } else {
        // 使用均匀分布确保数值分布无偏
        std::uniform_int_distribution<unsigned long long> dist(0, range - 1);
        return dist(engine);
    }
}
namespace Solution {
bool KDflag;
void KDTree(vector<Point>&p,int l,int r)
{
    if(l==r)return;
    int mid=l+(r-l)/2;
    nth_element(p.begin()+l,p.begin()+mid,p.begin()+r,[](Point a,Point b){
        if(KDflag)return a.x<b.x;
        else return a.y<b.y;
    });
    bool nowflag=KDflag;
    KDflag=(!nowflag);
    KDTree(p,l,mid);
    KDflag=(!nowflag);
    KDTree(p,mid+1,r);
}
}

//定义图
class Graph{
    bool Randflg;
    unsigned int PointNums;
public:vector<vector<pair<int,int>>>G;//first记录其目的点标号, second记录该边标号
    vector<Point>P;//存放点的信息，标号从1开始
    vector<Edge>E;//存放边的信息，标号从0开始
private:
    void addedge(int u,int v)
    {
        double dis=distance(P[u],P[v]);
        double rand_scale = myrand(RAND_MAX) * 1.0 / RAND_MAX + 1;  // [1.0, 2.0)
        int cap = static_cast<int>(rand_scale * dis / 20) + myrand(33) + 88;
        //int cap=(int)(myrand(RAND_MAX)*1.0/RAND_MAX+1)*(int)(dis)/20+myrand(37)+79;
        E.push_back(Edge(u,v,dis,cap,cap*1/2));
        G[u].push_back({v,(int)E.size()-1});
        E.push_back(Edge(v,u,dis,cap,cap*1/2));
        G[v].push_back({u,(int)E.size()-1});
    }
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
        int row =sqrt(n),col=n/row;
        n=row*col;
        PointNums=n;
        int sqn=pow(n,0.5);
        G.resize(n+1);
        P.resize(n+1);
        srand(time(0));
        int rlimit=RAND_MAX/row,climit=RAND_MAX/col;
        for(int i=1;i<=row;i++)
        {
            for(int j=1;j<=col;j++)
            {
                int now=j+(i-1)*col;
                P[now].x=myrand(rlimit)+(i-1)*rlimit;
                P[now].y=myrand(climit)+(j-1)*climit;
            }
        }
        for(int i=1;i<=n;i++)
        {
            P[i].x+=1.0 * myrand(RAND_MAX) / ((int)RAND_MAX), P[i].y += 1.0 * myrand(RAND_MAX) / ((int)RAND_MAX);
        }
        Solution::KDTree(P,1,n);
            int maxdis=max({distance(P[1],P[n]),distance(P[n/2],P[n]),distance(P[1],P[n/2])});
        maxdis=pow(maxdis,0.7);
            for(int i=2;i<=n;i++)
        {
            vector<pair<int,int>>vc;
                int st=max(1,i-(int)pow(n,0.6));
                for(int j=1;j<i;j++)
            {
                if(vc.size()<2)
                        vc.push_back({(int)distance(P[i],P[j]),j});
                else{
                    int dis=distance(P[i],P[j]);
                    if(dis<vc.front().first&&myrand(n)<sqn)
                        continue;
                    pair<int,int>now={dis,j};
                    for(int k=0;k<(int)vc.size();k++)
                    {
                        if(vc[k].first>dis)
                            swap(vc[k],now);
                    }
                    if(myrand(n)<sqn)
                        swap(now,vc.back());
                }
                if(!vc.empty()&&(myrand(n)<sqn))
                    vc.pop_back();
                if(j<st)
                    j+=pow(n,0.1);
            }
                for(auto [dis,to]:vc)
            {
                if(dis>maxdis&&myrand(n)>sqn)
                        continue;
                addedge(i,to);
            }
        }
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
