#ifndef MYGRAPHICSVIEW_H
#define MYGRAPHICSVIEW_H
#include <QObject>
#include <QGraphicsView>
#include <QPointF>

QT_BEGIN_NAMESPACE
class QWheelEvent;
QT_END_NAMESPACE

// 前向声明
class Graph;

class MyGraphicsView: public QGraphicsView
{
    Q_OBJECT
public:
    // 构造函数
    explicit MyGraphicsView(QWidget *parent=0);
    // 析构函数
    ~MyGraphicsView();
    // 鼠标是否按下
    bool isMousePressed=0;
    // 视图中心点，位置点
    QPointF centerAnchor,posAnchor;

    // 设置Graph指针，用于获取图的数据
    void setGraph(Graph* graph) { m_graph = graph; }

    // 获取当前缩放等级
    int getScalingOffset() const;

signals:
    // 添加鼠标点击信号
    void mouseClicked(QPointF scenePos);
    // 添加缩放变化信号
    void zoomChanged(int level);

protected:
    // 滚轮缩放
    void wheelEvent(QWheelEvent *event) override;
    // 上界
    void magnify();
    // 下界
    void shrink();
    // 放缩函数
    void scaling(qreal scaleFactor);
    // 鼠标按下
    void mousePressEvent(QMouseEvent *event) override;
    // 鼠标移动
    void mouseMoveEvent(QMouseEvent *event) override;
    // 鼠标释放
    void mouseReleaseEvent(QMouseEvent *event) override;
    // 添加鼠标双击事件处理
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    // 记录放缩累计，用于限制上下界
    int m_scalingOffset;

    // 图的指针，用于获取图的数据
    Graph* m_graph = nullptr;
};

#endif // MYGRAPHICSVIEW_H
