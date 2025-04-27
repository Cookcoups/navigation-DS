#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QGraphicsScene>
#include <QMainWindow>
#include <QTimer>
#include <QTextEdit>
#include <vector>
#include <unordered_map>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    //最近100个点
    void setnearpoint(double x, double y);
    //最短路
    void setshortestpath(int start, int end, int time);
    //刷新路径
    void updatepath();
    //刷新点
    void updatepoint();
    //更新地图可见性
    void updateMapVisibility();
    QGraphicsScene *m_scene;

private:
    //更新边的可见性
    void updateEdgeVisibility();

    Ui::MainWindow *ui;

    // 缩放性能优化相关变量
    int m_lastZoomLevel = -1; // 上次的缩放级别
    std::vector<bool> m_nodeVisibility; // 缓存节点可见性
    std::vector<int> m_nodeRepresentative; // 缓存节点代表
    std::unordered_map<int, std::vector<int>> m_visibilityCache; // 缓存不同缩放级别的可见节点
    bool m_visibilityCacheDirty = true; // 缓存是否需要重建
};
#endif // MAINWINDOW_H
