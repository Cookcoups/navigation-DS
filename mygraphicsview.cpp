#include"mygraphicsview.h"
#include <QWheelEvent>
#include <QPoint>
MyGraphicsView::MyGraphicsView(QWidget *parent)
    :QGraphicsView(parent)
{//放大倍数初始化为1
    m_scalingOffset=1;
}
MyGraphicsView::~MyGraphicsView(){

}
//上界判断
void MyGraphicsView::magnify(){
    if(m_scalingOffset>=10){
        m_scalingOffset=10;
        return;
    }
    m_scalingOffset+=1;
    scaling(1.25);
}
//下界判断
void MyGraphicsView::shrink(){
    if(m_scalingOffset<=1){
        m_scalingOffset=1;
        return;
    }
    m_scalingOffset-=1;
    scaling(0.8);
}

void MyGraphicsView::scaling(double scalefactor){
    scale(scalefactor,scalefactor);

}
void MyGraphicsView::wheelEvent(QWheelEvent *event){
    QPoint sroll=event->angleDelta();
    sroll.y()>0?magnify():shrink();
}
void MyGraphicsView::mousePressEvent(QMouseEvent *event){
    if(event->button()!=Qt::LeftButton)return;
    QGraphicsView::mousePressEvent(event);
    if(this->scene()==nullptr){
        return;
    }
    centerAnchor=mapToScene(event->pos())-event->pos()+QPointF(width()/2,height()/2);
    posAnchor=event->pos();
    isMousePressed=true;
}
void MyGraphicsView::mouseMoveEvent(QMouseEvent *event){
    QGraphicsView::mouseMoveEvent(event);
    QPointF offsetpos=event->pos()-posAnchor;
    if(isMousePressed){
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        centerOn(centerAnchor-offsetpos);
    }
}
void MyGraphicsView::mouseReleaseEvent(QMouseEvent *event){
    if(event->button()!=Qt::LeftButton)return;
    QGraphicsView::mouseReleaseEvent(event);
    isMousePressed=false;
}
