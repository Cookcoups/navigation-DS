#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QGraphicsScene>
#include <QMainWindow>

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
    void setnearpoint(double x,double y);
    //最短路
    void setshortestpath(int x,int y,int time);
    //刷新路径
    void updatepath();
    //刷新点
    void updatepoint();
    QGraphicsScene *m_scene;

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
