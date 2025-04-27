#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "basicstruct.h"
#include <QPainter>
#include <bits/stdc++.h>
#include <QMouseEvent>
#include <QGraphicsItem>
#include "mygraphicsview.h"
#include <QDebug>
#include <QColor>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QGroupBox>
#include <QTextEdit>
#include <set>
#include <limits>
#include <unordered_map>

const int pointnum = 10000; // 点数
bool vispath1[50005]; // 最短路占用边标记
bool vispath2[50005]; // 最近点占用边标记
bool vispoint1[10005]; // 最短路占用点标记
bool vispoint2[10005]; // 最近点占用点标记
QGraphicsLineItem *litem[50005]; // 边图元
QGraphicsEllipseItem *ellipseitem[50005]; // 点图元
Graph Gp(pointnum); // 构造图
MyGraphicsView *graphicsView; // 视图指针
std::vector<int> shortestpath; // 最短路存储
std::vector<int> nearpoint; // 最近点存储

// 全局变量用于记录选点
int selectedStartPoint = -1;
int selectedEndPoint = -1;

// 主窗口构造函数
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this); // ui绑定
    this->setWindowTitle("Navigation System");

    // 创建中央部件与布局
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    // 创建主布局
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    QHBoxLayout *topLayout = new QHBoxLayout();

    // 创建图形视图
    m_scene = new QGraphicsScene; // 场景创建
    graphicsView = new MyGraphicsView(this); // 视图创建绑定主窗口
    graphicsView->resize(800, 600); // 视图大小设定
    graphicsView->setScene(m_scene); // 场景塞进视图
    graphicsView->setSceneRect(0, 0, 800, 600); // 场景大小设定

    // 设置Graph指针，使MyGraphicsView能访问图数据
    graphicsView->setGraph(&Gp);

    // 创建控制面板
    QVBoxLayout *controlLayout = new QVBoxLayout();
    QGroupBox *controlGroup = new QGroupBox("Navigation Controls");
    QVBoxLayout *controlGroupLayout = new QVBoxLayout(controlGroup);

    // 起点终点显示
    QLabel *startPointLabel = new QLabel("Start Point: None");
    startPointLabel->setObjectName("startPointLabel");
    QLabel *endPointLabel = new QLabel("End Point: None");
    endPointLabel->setObjectName("endPointLabel");

    // 路径选择按钮
    QPushButton *calcShortestPathBtn = new QPushButton("Calculate Shortest Path (Distance)");
    QPushButton *calcFastestPathBtn = new QPushButton("Calculate Fastest Path (Time)");
    QPushButton *clearSelectionBtn = new QPushButton("Clear Selection");

    // 图例说明
    QGroupBox *legendGroup = new QGroupBox("Legend");
    QVBoxLayout *legendLayout = new QVBoxLayout(legendGroup);

    QLabel *purpleLegend = new QLabel("Purple: Selected Start/End Points");
    QLabel *pathLegend = new QLabel("Red: Path Points");
    QLabel *nearestLegend = new QLabel("Blue: Nearest 100 Points");
    QLabel *trafficLegend = new QLabel("Traffic Conditions:");
    QLabel *greenLegend = new QLabel("Green: Light Traffic");
    QLabel *yellowLegend = new QLabel("Yellow: Medium Traffic");
    QLabel *orangeLegend = new QLabel("Orange: Heavy Traffic");
    QLabel *redLegend = new QLabel("Dark Red: Congested");

    // 设置图例颜色样式
    purpleLegend->setStyleSheet("color: rgb(128, 0, 128); font-weight: bold;");
    pathLegend->setStyleSheet("color: red; font-weight: bold;");
    nearestLegend->setStyleSheet("color: rgb(30, 144, 255); font-weight: bold;");
    greenLegend->setStyleSheet("color: green;");
    yellowLegend->setStyleSheet("color: #CCCC00;");
    orangeLegend->setStyleSheet("color: orange;");
    redLegend->setStyleSheet("color: darkred;");

    // 添加图例到布局
    legendLayout->addWidget(purpleLegend);
    legendLayout->addWidget(pathLegend);
    legendLayout->addWidget(nearestLegend);
    legendLayout->addWidget(trafficLegend);
    legendLayout->addWidget(greenLegend);
    legendLayout->addWidget(yellowLegend);
    legendLayout->addWidget(orangeLegend);
    legendLayout->addWidget(redLegend);

    // 添加控件到控制面板
    controlGroupLayout->addWidget(startPointLabel);
    controlGroupLayout->addWidget(endPointLabel);
    controlGroupLayout->addWidget(calcShortestPathBtn);
    controlGroupLayout->addWidget(calcFastestPathBtn);
    controlGroupLayout->addWidget(clearSelectionBtn);

    // 添加控制面板和图例到右侧布局
    controlLayout->addWidget(controlGroup);
    controlLayout->addWidget(legendGroup);
    controlLayout->addStretch();

    // 创建底部显示区域，用于显示100个最近点的信息
    QTextEdit *nearestPointsTextEdit = new QTextEdit();
    nearestPointsTextEdit->setReadOnly(true);
    nearestPointsTextEdit->setMaximumHeight(100);
    nearestPointsTextEdit->setObjectName("nearestPointsTextEdit");

    // 将图形视图和控制面板添加到顶部布局
    topLayout->addWidget(graphicsView, 3);
    topLayout->addLayout(controlLayout, 1);

    // 将顶部布局和底部文本区域添加到主布局
    mainLayout->addLayout(topLayout, 5);
    mainLayout->addWidget(nearestPointsTextEdit, 1);

    // 连接按钮信号槽
    connect(calcShortestPathBtn, &QPushButton::clicked, this, [this]() {
        if (selectedStartPoint != -1 && selectedEndPoint != -1) {
            this->setshortestpath(selectedStartPoint, selectedEndPoint, 0); // 0 for distance-based
        } else {
            ui->statusbar->showMessage("Please select both start and end points");
        }
    });

    connect(calcFastestPathBtn, &QPushButton::clicked, this, [this]() {
        if (selectedStartPoint != -1 && selectedEndPoint != -1) {
            this->setshortestpath(selectedStartPoint, selectedEndPoint, 1); // 1 for time-based
        } else {
            ui->statusbar->showMessage("Please select both start and end points");
        }
    });

    connect(clearSelectionBtn, &QPushButton::clicked, this, [this, startPointLabel, endPointLabel]() {
        selectedStartPoint = -1;
        selectedEndPoint = -1;
        startPointLabel->setText("Start Point: None");
        endPointLabel->setText("End Point: None");

        // 清除可视化
        for(int i = 1; i <= pointnum; i++) {
            vispoint1[i] = false;
            vispoint2[i] = false;
        }
        for(int i = 0; i < Gp.GetEdgeNums(); i++) {
            vispath1[i] = false;
            vispath2[i] = false;
        }
        updatepoint();
        updatepath();

        // 更新地图可见性（根据当前缩放级别）
        updateMapVisibility();

        ui->statusbar->showMessage("Selection cleared");

        // 清空近邻点信息
        QTextEdit *nearestPointsText = findChild<QTextEdit*>("nearestPointsTextEdit");
        if (nearestPointsText) {
            nearestPointsText->clear();
        }
    });

    // 连接鼠标点击信号
    connect(graphicsView, &MyGraphicsView::mouseClicked, this, [this, startPointLabel, endPointLabel](QPointF scenePos) {
        // 鼠标点击时，在附近显示100个最近的点
        setnearpoint(scenePos.x(), scenePos.y());

        // 找到最近的点作为选中点
        if (!nearpoint.empty()) {
            int closestPoint = nearpoint[0]; // 最近的点

            if (selectedStartPoint == -1) {
                // 设置起点
                selectedStartPoint = closestPoint;
                startPointLabel->setText(QString("Start Point: %1").arg(selectedStartPoint));
                ui->statusbar->showMessage(QString("Start point selected: %1").arg(selectedStartPoint));
            } else if (selectedEndPoint == -1) {
                // 设置终点
                selectedEndPoint = closestPoint;
                endPointLabel->setText(QString("End Point: %1").arg(selectedEndPoint));
                ui->statusbar->showMessage(QString("End point selected: %1").arg(selectedEndPoint));
            } else {
                // 如果起点和终点都已设置，则重置为新的起点
                selectedStartPoint = closestPoint;
                selectedEndPoint = -1;
                startPointLabel->setText(QString("Start Point: %1").arg(selectedStartPoint));
                endPointLabel->setText("End Point: None");
                ui->statusbar->showMessage(QString("New start point selected: %1").arg(selectedStartPoint));
            }
            // 在选择点后立即更新点的显示，使紫色标记生效
            updatepoint();

            // 更新地图可见性，确保选中的点可见
            updateMapVisibility();
        }
    });

    // 连接缩放变化信号
    connect(graphicsView, &MyGraphicsView::zoomChanged, this, [this](int level) {
        // 当缩放级别变化时，更新地图可见性
        updateMapVisibility();
    });

    srand(time(0));
    Gp.StartRandFlow(); // 随机边流量线程启动
    auto edgenum = Gp.GetEdgeNums(); // 获取边数

    // 初始化边图元并塞入场景
    for(int i = 0; i < (int)edgenum; i++) {
        litem[i] = new QGraphicsLineItem;
        QGraphicsLineItem *item = litem[i];
        item->setPos(0, 0);
        // 除50适应实际大小
        item->setLine(Gp.P[Gp.E[i].P1].x/50, Gp.P[Gp.E[i].P1].y/50, Gp.P[Gp.E[i].P2].x/50, Gp.P[Gp.E[i].P2].y/50);
        m_scene->addItem(item); // 加入边图元
    }

    // 刷新边流量颜色
    updatepath();

    // 初始化点塞入场景
    for(int i = 1; i <= pointnum; i++) {
        ellipseitem[i] = new QGraphicsEllipseItem;
        QGraphicsEllipseItem *item = ellipseitem[i];
        item->setPos(0, 0);
        item->setPen(QPen(Qt::black));
        item->setBrush(Qt::black);
        item->setRect(QRectF(Gp.P[i].x/50, Gp.P[i].y/50, 1, 1));
        m_scene->addItem(item);
    }

    // 启动定时器以定期更新流量显示
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        updatepath();
        // 如果处于放大状态，确保更新地图可见性
        if (graphicsView->getScalingOffset() > 1) {
            updateMapVisibility();
        }
    });
    timer->start(1000); // 每秒更新一次

    // 设置窗口大小
    resize(1200, 800);
}

MainWindow::~MainWindow()
{
    // 停止随机流量线程
    Gp.StopRandFlow();
    delete ui;
}

// Implementation of the setnearpoint function to find and display the 100 nearest points
void MainWindow::setnearpoint(double x, double y) {
    // 标记缓存需要更新
    m_visibilityCacheDirty = true;

    // Clear previous nearest points visualization
    for(int i = 1; i <= pointnum; i++) {
        vispoint2[i] = false;
    }
    for(int i = 0; i < Gp.GetEdgeNums(); i++) {
        vispath2[i] = false;
    }

    // Create a point from the given coordinates
    Point p;
    p.x = x * 50; // Scale up to match the graph scale
    p.y = y * 50;

    // Get 100 nearest points
    nearpoint = Gp.get100points(p);

    // 打印调试信息，确认找到了100个最近的点
    QString pointsList = "Nearest 100 Points: ";
    int count = 0;

    // 确保我们有100个点（或附近最多的点）
    int numPoints = nearpoint.size();
    ui->statusbar->showMessage(QString("Found %1 nearest points").arg(numPoints));

    // Mark the points and edges for visualization
    for(int i = 0; i < nearpoint.size(); i++) {
        int pointId = nearpoint[i];
        vispoint2[pointId] = true; // 确保正确设置标记

        pointsList += QString::number(pointId);
        if (i < nearpoint.size() - 1) {
            pointsList += ", ";
        }
        count++;
        // 每20个点换一行，提高可读性
        if (count % 20 == 0 && i < nearpoint.size() - 1) {
            pointsList += "<br>";
        }

        // Mark edges connected to the nearest points
        for(auto [to, edge] : Gp.G[pointId]) {
            if(std::find(nearpoint.begin(), nearpoint.end(), to) != nearpoint.end()) {
                vispath2[edge] = true;
            }
        }
    }

    // 显示最近100个点的编号
    QTextEdit *nearestPointsText = findChild<QTextEdit*>("nearestPointsTextEdit");
    if (nearestPointsText) {
        nearestPointsText->setHtml(pointsList);
    }

    // Update visualization
    updatepoint();
    updatepath();

    // 确保所选点和附近100个点在缩放时保持可见
    updateMapVisibility();
}

// Implementation of setshortestpath to calculate and display the shortest path
void MainWindow::setshortestpath(int start, int end, int time) {
    // 标记缓存需要更新
    m_visibilityCacheDirty = true;

    // 确保选择的是实际的点而不是附近100个点中的某个点
    if (start <= 0 || start > pointnum || end <= 0 || end > pointnum) {
        ui->statusbar->showMessage("Invalid start or end point selected");
        return;
    }

    // Clear previous shortest path visualization
    for(int i = 1; i <= pointnum; i++) {
        vispoint1[i] = false;
    }
    for(int i = 0; i < Gp.GetEdgeNums(); i++) {
        vispath1[i] = false;
    }

    // Calculate shortest path directly between selected points
    double distance = 0, total_time = 0;
    shortestpath = Gp.dij(start, end, &Gp, time, distance, total_time);

    if(!shortestpath.empty()) {
        // Mark points on the path
        int cur = start;
        vispoint1[cur] = true;

        for(int i = 0; i < (int)shortestpath.size(); i++) {
            int edge = shortestpath[i];
            vispath1[edge] = true;
            vispath1[edge ^ 1] = true; // Also mark the reverse edge

            // Update current point
            if(Gp.E[edge].P1 == cur) {
                cur = Gp.E[edge].P2;
            } else {
                cur = Gp.E[edge].P1;
            }
            vispoint1[cur] = true;
        }

        // Display information about the path
        QString info;
        if(time == 1) {
            info = QString("Fastest Path from %1 to %2 - Distance: %3 units, Time: %4 minutes")
            .arg(start)
                .arg(end)
                .arg(distance, 0, 'f', 2)
                .arg(total_time * 60, 0, 'f', 2); // Convert to minutes for better readability
        } else {
            info = QString("Shortest Path from %1 to %2 - Distance: %3 units, Time: %4 minutes")
            .arg(start)
                .arg(end)
                .arg(distance, 0, 'f', 2)
                .arg(total_time * 60, 0, 'f', 2);
        }
        ui->statusbar->showMessage(info);
    } else {
        ui->statusbar->showMessage(QString("No path found between points %1 and %2").arg(start).arg(end));
    }

    // Update visualization
    updatepoint();
    updatepath();

    // 确保路径上的点和边在缩放状态下可见
    updateMapVisibility();
}

// 优化后的 updateMapVisibility 函数
void MainWindow::updateMapVisibility() {
    int zoomLevel = graphicsView->getScalingOffset();

    // 如果缩放级别没变化且没有选择新的起点终点，可以使用缓存
    bool canUseCache = (zoomLevel == m_lastZoomLevel && !m_visibilityCacheDirty);

    // 检查是否有状态变化需要重建缓存
    if (m_nodeVisibility.size() != pointnum + 1 || m_visibilityCacheDirty) {
        // 初始化或重置缓存
        m_nodeVisibility.resize(pointnum + 1, false);
        m_nodeRepresentative.resize(pointnum + 1, -1);
        canUseCache = false;
    }

    // 如果缩放变化，重新计算可见性
    if (zoomLevel != m_lastZoomLevel) {
        canUseCache = false;
        m_lastZoomLevel = zoomLevel;
    }

    // 检查缓存中是否有当前缩放级别的数据
    if (canUseCache && m_visibilityCache.find(zoomLevel) != m_visibilityCache.end()) {
        // 使用缓存的可见性数据
        auto& visibleNodes = m_visibilityCache[zoomLevel];

        // 先隐藏所有点和边
        for (int i = 1; i <= pointnum; i++) {
            ellipseitem[i]->setVisible(false);
        }
        for (int i = 0; i < Gp.GetEdgeNums(); i++) {
            litem[i]->setVisible(false);
        }

        // 显示缓存中的可见节点
        for (int i : visibleNodes) {
            ellipseitem[i]->setVisible(true);
        }

        // 处理边的可见性
        updateEdgeVisibility();

        // 更新点的大小
        updatepoint();
        return;
    }

    // 计算新的可见性数据
    // 计算可见性比例
    double visibilityRatio;
    if (zoomLevel <= 5) {
        visibilityRatio = 0.33 + (zoomLevel - 1) * (0.34 / 4);
    } else {
        visibilityRatio = 0.67 + (zoomLevel - 5) * (0.33 / 5);
    }

    int skipFactor = std::max(1, static_cast<int>(1.0 / visibilityRatio));

    // 隐藏所有点和边
    for (int i = 1; i <= pointnum; i++) {
        ellipseitem[i]->setVisible(false);
        m_nodeVisibility[i] = false;
    }
    for (int i = 0; i < Gp.GetEdgeNums(); i++) {
        litem[i]->setVisible(false);
    }

    // 创建当前缩放级别的可见节点列表
    std::vector<int> visibleNodes;

    // 标记应该可见的节点
    for (int i = 1; i <= pointnum; i++) {
        // 特殊点始终可见
        if (vispoint1[i] || vispoint2[i] || i == selectedStartPoint || i == selectedEndPoint) {
            m_nodeVisibility[i] = true;
            ellipseitem[i]->setVisible(true);
            visibleNodes.push_back(i);
        }
        // 常规点按比例显示
        else if (i % skipFactor == 0 || zoomLevel >= 10) {
            m_nodeVisibility[i] = true;
            ellipseitem[i]->setVisible(true);
            visibleNodes.push_back(i);
        }
    }

    // 计算隐藏点的代表 - 使用网格分区优化
    const int gridSize = 50; // 网格大小，可根据实际情况调整
    std::unordered_map<int, std::vector<int>> grid; // 网格中的可见点

    // 将可见点放入网格
    for (int i : visibleNodes) {
        int gridX = static_cast<int>(Gp.P[i].x / (50 * gridSize));
        int gridY = static_cast<int>(Gp.P[i].y / (50 * gridSize));
        int gridKey = gridX * 10000 + gridY; // 简单哈希生成网格键
        grid[gridKey].push_back(i);
    }

    // 为每个隐藏点找到最近的可见点
    for (int i = 1; i <= pointnum; i++) {
        if (m_nodeVisibility[i]) {
            // 可见点代表自身
            m_nodeRepresentative[i] = i;
        } else {
            // 计算该点所在的网格和相邻网格
            int gridX = static_cast<int>(Gp.P[i].x / (50 * gridSize));
            int gridY = static_cast<int>(Gp.P[i].y / (50 * gridSize));

            double minDist = std::numeric_limits<double>::max();
            int closestNode = -1;

            // 搜索当前网格和相邻网格
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int neighborGridKey = (gridX + dx) * 10000 + (gridY + dy);
                    if (grid.find(neighborGridKey) != grid.end()) {
                        for (int visibleNode : grid[neighborGridKey]) {
                            double dist = distance(Gp.P[i], Gp.P[visibleNode]);
                            if (dist < minDist) {
                                minDist = dist;
                                closestNode = visibleNode;
                            }
                        }
                    }
                }
            }

            // 如果在相邻网格中没找到，搜索所有可见点
            if (closestNode == -1 && !visibleNodes.empty()) {
                // 回退策略：搜索所有可见点
                for (int visibleNode : visibleNodes) {
                    double dist = distance(Gp.P[i], Gp.P[visibleNode]);
                    if (dist < minDist) {
                        minDist = dist;
                        closestNode = visibleNode;
                    }
                }
            }

            m_nodeRepresentative[i] = closestNode;
        }
    }

    // 存储缓存
    m_visibilityCache[zoomLevel] = visibleNodes;
    m_visibilityCacheDirty = false;

    // 更新边的可见性
    updateEdgeVisibility();

    // 更新点的大小
    updatepoint();
}

// 新增的 updateEdgeVisibility 函数
void MainWindow::updateEdgeVisibility() {
    // 用于跟踪哪些点对之间已有可见边
    std::set<std::pair<int, int>> visibleEdgePairs;

    // 首先处理特殊边（路径边和最近点边）
    for (int i = 0; i < Gp.GetEdgeNums(); i++) {
        if (vispath1[i] || vispath2[i]) {
            int p1 = Gp.E[i].P1;
            int p2 = Gp.E[i].P2;

            // 确保端点可见
            ellipseitem[p1]->setVisible(true);
            ellipseitem[p2]->setVisible(true);
            m_nodeVisibility[p1] = true;
            m_nodeVisibility[p2] = true;

            // 显示边
            litem[i]->setVisible(true);

            // 记录这对点已有可见边
            int minP = std::min(p1, p2);
            int maxP = std::max(p1, p2);
            visibleEdgePairs.insert({minP, maxP});
        }
    }

    // 处理常规边
    for (int i = 0; i < Gp.GetEdgeNums(); i += 2) { // 优化：每次处理一对边（正向和反向）
        if (vispath1[i] || vispath2[i]) {
            // 已处理过的特殊边
            continue;
        }

        int p1 = Gp.E[i].P1;
        int p2 = Gp.E[i].P2;

        // 获取点的代表
        int rep1 = m_nodeRepresentative[p1];
        int rep2 = m_nodeRepresentative[p2];

        // 跳过无效代表或自循环
        if (rep1 == -1 || rep2 == -1 || rep1 == rep2) {
            continue;
        }

        // 确保点对有序
        int minP = std::min(rep1, rep2);
        int maxP = std::max(rep1, rep2);
        std::pair<int, int> edgePair = {minP, maxP};

        // 检查是否已有这对代表点之间的边
        if (visibleEdgePairs.find(edgePair) == visibleEdgePairs.end()) {
            // 显示这条边
            litem[i]->setLine(
                Gp.P[rep1].x/50, Gp.P[rep1].y/50,
                Gp.P[rep2].x/50, Gp.P[rep2].y/50
                );
            litem[i]->setVisible(true);

            // 记录已显示的边
            visibleEdgePairs.insert(edgePair);
        }
    }
}

// Implementation of updatepath to refresh the visualization of edges
void MainWindow::updatepath() {
    for(int i = 0; i < Gp.GetEdgeNums(); i++) {
        QGraphicsLineItem *item = litem[i];

        // 只处理可见的边
        if (!item->isVisible()) continue;

        // Calculate traffic intensity (ratio of flow to capacity)
        double intensity = 1.0 * Gp.E[i].flow / Gp.E[i].capacity;

        // Set pen based on traffic condition
        QPen pen;

        // 调整线条宽度 - 放大时稍微增加线宽
        double lineWidth = 1.0;
        int zoomLevel = graphicsView->getScalingOffset();
        if (zoomLevel > 1) {
            lineWidth = std::min(2.0, 1.0 + zoomLevel * 0.1);
        }

        if(vispath1[i]) {
            // Shortest path edges - highlighted in red and made thicker
            pen.setColor(Qt::red);
            pen.setWidth(3 * lineWidth); // 加粗显示最短路径
        } else if(vispath2[i]) {
            // Edges between nearest 100 points
            pen.setColor(QColor(30, 144, 255)); // 使用更亮的蓝色
            pen.setWidth(lineWidth);
        } else {
            // Regular edges with traffic intensity coloring
            if(intensity < 0.5) {
                // Light traffic - green
                pen.setColor(QColor(0, 255, 0));
            }
            else if(intensity < 0.8) {
                // Medium traffic - yellow
                pen.setColor(QColor(255, 255, 0));
            } else if(intensity < 1.0) {
                // Heavy traffic - orange
                pen.setColor(QColor(255, 165, 0));
            } else {
                // Congested - dark red
                pen.setColor(QColor(139, 0, 0));
            }
            pen.setWidth(lineWidth);
        }

        // Set pen to item
        item->setPen(pen);
    }
}

void MainWindow::updatepoint() {
    int zoomLevel = graphicsView->getScalingOffset();
    double baseSize = 1.0;

    // 放大时点显示加大，但不要过大
    if (zoomLevel > 1) {
        baseSize = std::min(2.0, 1.0 + zoomLevel * 0.1);
    }

    for(int i = 1; i <= pointnum; i++) {
        QGraphicsEllipseItem *item = ellipseitem[i];

        // 只处理可见的点，减少计算量
        if (!item->isVisible()) continue;

        // 如果是用户选择的起点或终点，使用紫色标记
        if(i == selectedStartPoint || i == selectedEndPoint) {
            item->setPen(QPen(QColor(128, 0, 128))); // 紫色
            item->setBrush(QColor(128, 0, 128));
            item->setRect(QRectF(Gp.P[i].x/50 - 4 * baseSize, Gp.P[i].y/50 - 4 * baseSize, 8 * baseSize, 8 * baseSize)); // 比路径点更大
        }
        else if(vispoint1[i]) {
            // Points on shortest path - highlighted in red and enlarged
            item->setPen(QPen(Qt::red));
            item->setBrush(Qt::red);
            item->setRect(QRectF(Gp.P[i].x/50 - 3 * baseSize, Gp.P[i].y/50 - 3 * baseSize, 6 * baseSize, 6 * baseSize)); // 路径点
        } else if(vispoint2[i]) {
            // Points in nearest 100 - make them bright blue and LARGER for visibility
            item->setPen(QPen(QColor(30, 144, 255))); // 使用更亮的蓝色
            item->setBrush(QColor(30, 144, 255));
            item->setRect(QRectF(Gp.P[i].x/50 - 2 * baseSize, Gp.P[i].y/50 - 2 * baseSize, 4 * baseSize, 4 * baseSize)); // 附近点
        } else {
            // Regular points
            item->setPen(QPen(Qt::black));
            item->setBrush(Qt::black);
            item->setRect(QRectF(Gp.P[i].x/50 - baseSize/2, Gp.P[i].y/50 - baseSize/2, baseSize, baseSize));
        }
    }
}
