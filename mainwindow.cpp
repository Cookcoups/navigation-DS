#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "basicstruct.h"
#include <QPainter>
#include <bits/stdc++.h>
#include <QMouseEvent>
#include<QGraphicsItem>
#include"mygraphicsview.h"
//#include <QDebug>
#include <QColor>
const int pointnum=10000;//点数
bool vispath1[50005];//最短路占用边标记
bool vispath2[50005];//最近点占用边标记
bool vispoint1[10005];//最短路占用点标记
bool vispoint2[10005];//最近点占用点标记
QGraphicsLineItem *litem[50005];//边图元
QGraphicsEllipseItem *ellipseitem[50005];//点图元
Graph Gp(pointnum);//构造图
MyGraphicsView *graphicsView;//视图指针
std::vector<int> shortestpath;//最短路存储
std::vector<int> nearpoint;//最近点存储

//主窗口构造函数
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);//ui绑定
    m_scene=new QGraphicsScene;//场景创建
    graphicsView=new MyGraphicsView(this);//视图创建绑定主窗口
    graphicsView->resize(600,600);//视图大小设定
    graphicsView->setScene(m_scene);//场景塞进视图
    graphicsView->setSceneRect(0,0,600,600);//场景大小设定
    srand(time(0));
    Gp.StartRandFlow();//随机边流量线程启动
    auto edgenum=Gp.GetEdgeNums();//获取边数
    //初始化边图元并塞入场景
    for(int i=0;i<(int)edgenum;i++)
    {
        litem[i]=new QGraphicsLineItem;
        QGraphicsLineItem *item=litem[i];
        item->setPos(0,0);
        //除50适应实际大小
        item->setLine(Gp.P[Gp.E[i].P1].x/50,Gp.P[Gp.E[i].P1].y/50,Gp.P[Gp.E[i].P2].x/50,Gp.P[Gp.E[i].P2].y/50);
        m_scene->addItem(item);//加入边图元
    }
    //刷新边流量
    //updatepath();
    //初始化点塞入场景
    for(int i=1;i<=pointnum;i++)
    {
        ellipseitem[i]=new QGraphicsEllipseItem;
        QGraphicsEllipseItem *item=ellipseitem[i];
        item->setPos(0,0);
        item->setPen(QPen(Qt::black));
        item->setBrush(Qt::black);
        item->setRect(QRect(Gp.P[i].x/50,Gp.P[i].y/50,1,1));
        m_scene->addItem(item);
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}
