#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QMouseEvent>
#include <QStatusBar>
#include <QTextStream>
#include <QFileInfo>
#include <QRegularExpression>
#include <rclcpp/rclcpp.hpp>
#include <QInputDialog>

class GraphEditor;  // Forward declaration

class CustomGraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    CustomGraphicsView(QGraphicsScene *scene, GraphEditor *parent);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    GraphEditor *editor;
};

class GraphEditor : public QMainWindow {
    Q_OBJECT
public:
    GraphEditor(QWidget *parent = nullptr);
    void handleMousePress(QMouseEvent *event);
    void updateMousePosition(const QPointF &pos);

private slots:
    void loadMap();
    void loadGraph();
    void setAddVertexMode();
    void setAddEdgeMode();
    void setEraseMode();
    void saveGraph();
    void subdivideGraph();
    void handleResize();  // 새로운 슬롯 추가

private:
    enum class Mode {
        None,
        AddVertex,
        AddEdge,
        Erase
    };

    void addVertex(const QPointF &pos);
    void handleEdgeCreation(const QPointF &pos);
    void handleErase(const QPointF &pos);
    void subdivideEdges(double threshold);

    // UI 요소들
    QGraphicsScene *scene;
    CustomGraphicsView *view;
    QPushButton *loadMapBtn;
    QPushButton *addVertexBtn;
    QPushButton *addEdgeBtn;
    QPushButton *eraseBtn;
    QPushButton *saveGraphBtn;
    QPushButton *subdivideBtn;
    
    // 상태 변수들
    Mode currentMode;
    QGraphicsEllipseItem *selectedVertex;
    QList<QGraphicsEllipseItem*> vertices;
    QList<QGraphicsLineItem*> edges;
    QLabel *coordLabel;  // 좌표 표시 라벨
    rclcpp::Node::SharedPtr node_;  // ROS 노드 추가
}; 