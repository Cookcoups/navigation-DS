#include "mygraphicsview.h"
#include "basicstruct.h"
#include <QWheelEvent>
#include <QMouseEvent>
#include <QPoint>
#include <QPointF>
#include <QDebug>

MyGraphicsView::MyGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
{
    // 放大倍数初始化为1
    m_scalingOffset = 1;

    // 设置视图属性
    setDragMode(QGraphicsView::NoDrag); // 禁用默认拖动模式
    setRenderHint(QPainter::Antialiasing); // 抗锯齿
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
}

MyGraphicsView::~MyGraphicsView() {
}

// 获取缩放级别
int MyGraphicsView::getScalingOffset() const {
    return m_scalingOffset;
}

// 上界判断
void MyGraphicsView::magnify() {
    if(m_scalingOffset >= 10) {
        m_scalingOffset = 10;
        return;
    }
    m_scalingOffset += 1;
    scaling(1.25);

    // 发出缩放变化信号
    emit zoomChanged(m_scalingOffset);
}

// 下界判断
void MyGraphicsView::shrink() {
    if(m_scalingOffset <= 1) {
        m_scalingOffset = 1;
        return;
    }
    m_scalingOffset -= 1;
    scaling(0.8);

    // 发出缩放变化信号
    emit zoomChanged(m_scalingOffset);
}

void MyGraphicsView::scaling(double scalefactor) {
    scale(scalefactor, scalefactor);
}

void MyGraphicsView::wheelEvent(QWheelEvent *event) {
    QPoint scroll = event->angleDelta();
    scroll.y() > 0 ? magnify() : shrink();
}

void MyGraphicsView::mousePressEvent(QMouseEvent *event) {
    if(event->button() != Qt::LeftButton) return;
    QGraphicsView::mousePressEvent(event);

    if(this->scene() == nullptr) {
        return;
    }

    centerAnchor = mapToScene(event->pos()) - event->pos() + QPointF(width()/2, height()/2);
    posAnchor = event->pos();
    isMousePressed = true;
}

void MyGraphicsView::mouseMoveEvent(QMouseEvent *event) {
    QGraphicsView::mouseMoveEvent(event);

    if(isMousePressed) {
        QPointF offsetpos = event->pos() - posAnchor;
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        centerOn(centerAnchor - offsetpos);
    }
}

void MyGraphicsView::mouseReleaseEvent(QMouseEvent *event) {
    if(event->button() != Qt::LeftButton) return;
    QGraphicsView::mouseReleaseEvent(event);
    isMousePressed = false;
}

void MyGraphicsView::mouseDoubleClickEvent(QMouseEvent *event) {
    QGraphicsView::mouseDoubleClickEvent(event);

    if(event->button() == Qt::LeftButton) {
        // Convert mouse position to scene coordinates
        QPointF scenePos = mapToScene(event->pos());

        // Emit signal with scene position
        emit mouseClicked(scenePos);
    }
}
