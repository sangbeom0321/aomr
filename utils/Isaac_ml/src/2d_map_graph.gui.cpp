#include "2d_map_graph.gui.hpp"
#include <QApplication>

CustomGraphicsView::CustomGraphicsView(QGraphicsScene *scene, GraphEditor *parent)
    : QGraphicsView(scene), editor(parent) {
    setMouseTracking(true);
}

void CustomGraphicsView::mousePressEvent(QMouseEvent *event) {
    editor->handleMousePress(event);
}

void CustomGraphicsView::mouseMoveEvent(QMouseEvent *event) {
    editor->updateMousePosition(mapToScene(event->pos()));
    QGraphicsView::mouseMoveEvent(event);
}

GraphEditor::GraphEditor(QWidget *parent) : QMainWindow(parent) {
    // ROS 노드 초기화
    node_ = std::make_shared<rclcpp::Node>("graph_editor");
    
    // 중앙 위젯 설정
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    
    // 레이아웃 설정
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);
    
    // 그래픽스 뷰 설정
    scene = new QGraphicsScene(this);
    view = new CustomGraphicsView(scene, this);
    mainLayout->addWidget(view);
    
    // 버튼들 정의
    loadMapBtn = new QPushButton("Load Map", this);
    addVertexBtn = new QPushButton("Add Vertex", this);
    addEdgeBtn = new QPushButton("Add Edge", this);
    eraseBtn = new QPushButton("Erase", this);
    saveGraphBtn = new QPushButton("Save Graph", this);
    subdivideBtn = new QPushButton("Subdivide Edges", this);
    QPushButton *loadGraphBtn = new QPushButton("Load Graph", this);
    
    // 오른쪽 컨트롤 패널
    QVBoxLayout *controlLayout = new QVBoxLayout();
    mainLayout->addLayout(controlLayout);
    
    controlLayout->addWidget(loadMapBtn);
    controlLayout->addWidget(loadGraphBtn);
    controlLayout->addWidget(addVertexBtn);
    controlLayout->addWidget(addEdgeBtn);
    controlLayout->addWidget(eraseBtn);
    controlLayout->addWidget(subdivideBtn);
    controlLayout->addWidget(saveGraphBtn);
    controlLayout->addStretch();
    
    // 버튼 연결
    connect(loadMapBtn, &QPushButton::clicked, this, &GraphEditor::loadMap);
    connect(loadGraphBtn, &QPushButton::clicked, this, &GraphEditor::loadGraph);
    connect(addVertexBtn, &QPushButton::clicked, this, &GraphEditor::setAddVertexMode);
    connect(addEdgeBtn, &QPushButton::clicked, this, &GraphEditor::setAddEdgeMode);
    connect(eraseBtn, &QPushButton::clicked, this, &GraphEditor::setEraseMode);
    connect(saveGraphBtn, &QPushButton::clicked, this, &GraphEditor::saveGraph);
    connect(subdivideBtn, &QPushButton::clicked, this, &GraphEditor::subdivideGraph);
    
    // 좌표 표시 라벨 추가
    coordLabel = new QLabel(this);
    coordLabel->setAlignment(Qt::AlignRight);
    coordLabel->setMinimumWidth(200);
    statusBar()->addPermanentWidget(coordLabel);
    
    setMinimumSize(800, 600);
    currentMode = Mode::None;
    selectedVertex = nullptr;
    
    // 뷰 설정
    view->setRenderHint(QPainter::Antialiasing);
    view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setDragMode(QGraphicsView::NoDrag);  // 드래그 모드 비활성화

    // 윈도우 리사이즈 이벤트 처리
    connect(view->viewport(), SIGNAL(resize()), this, SLOT(handleResize()));
}

void GraphEditor::handleResize() {
    if (!scene->items().isEmpty()) {
        view->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    }
}

void GraphEditor::loadMap() {
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Map Image"), "", tr("Image Files (*.png *.jpg *.bmp *.pgm)"));
        
    if (!fileName.isEmpty()) {
        QImage image(fileName);
        if (!image.isNull()) {
            scene->clear();
            vertices.clear();
            edges.clear();

            // 이미지 추가
            QGraphicsPixmapItem* pixmapItem = scene->addPixmap(QPixmap::fromImage(image));
            scene->setSceneRect(pixmapItem->boundingRect());

            // 뷰 설정
            view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            view->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
        }
    }
}

void GraphEditor::setAddVertexMode() {
    currentMode = Mode::AddVertex;
    view->setCursor(Qt::CrossCursor);
}

void GraphEditor::setAddEdgeMode() {
    currentMode = Mode::AddEdge;
    view->setCursor(Qt::CrossCursor);
    selectedVertex = nullptr;
}

void GraphEditor::saveGraph() {
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Save Graph"), "", tr("YAML files (*.yaml)"));
    
    if (fileName.isEmpty()) return;
    
    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly)) {
        QTextStream stream(&file);
        
        // YAML 헤더
        stream << "graph:\n";
        
        // 정점 정보 저장
        stream << "  vertices:\n";
        for (int i = 0; i < vertices.size(); ++i) {
            QPointF center = vertices[i]->sceneBoundingRect().center();
            stream << "    " << i << ":\n";
            stream << "      x: " << static_cast<int>(center.x()) << "\n";
            stream << "      y: " << static_cast<int>(center.y()) << "\n";
            
            // 이 정점과 연결된 모든 간선 찾기
            QList<int> connections;
            for (const auto& edge : edges) {
                QLineF line = edge->line();
                QPointF vertexCenter = vertices[i]->sceneBoundingRect().center();
                
                // 부동소수점 비교를 위한 근사값 비교
                auto pointsEqual = [](const QPointF& p1, const QPointF& p2) {
                    return (qAbs(p1.x() - p2.x()) < 0.1) && (qAbs(p1.y() - p2.y()) < 0.1);
                };
                
                if (pointsEqual(line.p1(), vertexCenter)) {
                    for (int j = 0; j < vertices.size(); ++j) {
                        if (i != j && pointsEqual(vertices[j]->sceneBoundingRect().center(), line.p2())) {
                            connections.append(j);
                            break;
                        }
                    }
                } else if (pointsEqual(line.p2(), vertexCenter)) {
                    for (int j = 0; j < vertices.size(); ++j) {
                        if (i != j && pointsEqual(vertices[j]->sceneBoundingRect().center(), line.p1())) {
                            connections.append(j);
                            break;
                        }
                    }
                }
            }
            
            // 연결된 정점들 저장
            stream << "      connections: [";
            for (int j = 0; j < connections.size(); ++j) {
                stream << connections[j];
                if (j < connections.size() - 1) {
                    stream << ", ";
                }
            }
            stream << "]\n";
        }
        
        file.close();
    }
}

void GraphEditor::setEraseMode() {
    currentMode = Mode::Erase;
    view->setCursor(Qt::ForbiddenCursor);
    if (selectedVertex) {
        selectedVertex->setBrush(QBrush(Qt::red));
        selectedVertex = nullptr;
    }
}

void GraphEditor::handleMousePress(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        QPointF scenePos = view->mapToScene(event->pos());
        
        switch(currentMode) {
            case Mode::AddVertex:
                addVertex(scenePos);
                break;
            case Mode::AddEdge:
                handleEdgeCreation(scenePos);
                break;
            case Mode::Erase:
                handleErase(scenePos);
                break;
            default:
                break;
        }
    }
}

void GraphEditor::addVertex(const QPointF &pos) {
    QGraphicsEllipseItem *vertex = scene->addEllipse(
        pos.x()-5, pos.y()-5, 10, 10,
        QPen(Qt::black), QBrush(Qt::red));
    vertex->setFlag(QGraphicsItem::ItemIsMovable);
    vertex->setFlag(QGraphicsItem::ItemIsSelectable);
    
    // 툴팁 설정 (현재 정점의 인덱스)
    int vertexIndex = vertices.size();
    vertex->setToolTip(QString("Vertex %1").arg(vertexIndex));
    
    // 정점 번호 텍스트 추가
    QGraphicsTextItem *label = scene->addText(QString::number(vertexIndex));
    label->setDefaultTextColor(Qt::black);
    label->setPos(pos.x() + 8, pos.y() - 8);
    
    // 정점과 라벨을 그룹화
    QGraphicsItemGroup *group = scene->createItemGroup({vertex, label});
    group->setFlag(QGraphicsItem::ItemIsMovable);
    
    vertices.append(vertex);
}

void GraphEditor::handleEdgeCreation(const QPointF &pos) {
    QGraphicsEllipseItem *clickedVertex = nullptr;
    
    // 한 위치에서 가장 가까운 정점 찾기
    for (auto vertex : vertices) {
        if (vertex->contains(vertex->mapFromScene(pos))) {
            clickedVertex = vertex;
            break;
        }
    }
    
    if (clickedVertex) {
        if (!selectedVertex) {
            selectedVertex = clickedVertex;
            selectedVertex->setBrush(QBrush(Qt::green));
        } else {
            if (selectedVertex != clickedVertex) {
                // 간선 추가
                QLineF line(selectedVertex->sceneBoundingRect().center(),
                          clickedVertex->sceneBoundingRect().center());
                edges.append(scene->addLine(line, QPen(Qt::black, 2)));
            }
            selectedVertex->setBrush(QBrush(Qt::red));
            selectedVertex = nullptr;
        }
    }
}

void GraphEditor::handleErase(const QPointF &pos) {
    // 정점 삭제
    for (auto it = vertices.begin(); it != vertices.end(); ) {
        if ((*it)->contains((*it)->mapFromScene(pos))) {
            // 해당 정점과 연결된 모든 간선 찾아서 삭제
            for (auto edgeIt = edges.begin(); edgeIt != edges.end(); ) {
                QLineF line = (*edgeIt)->line();
                QPointF center = (*it)->sceneBoundingRect().center();
                
                if (line.p1() == center || line.p2() == center) {
                    scene->removeItem(*edgeIt);
                    delete *edgeIt;
                    edgeIt = edges.erase(edgeIt);
                } else {
                    ++edgeIt;
                }
            }
            
            // 정점의 라벨 찾아서 삭제
            QList<QGraphicsItem*> items = scene->items((*it)->sceneBoundingRect());
            for (QGraphicsItem* item : items) {
                if (QGraphicsTextItem* textItem = qgraphicsitem_cast<QGraphicsTextItem*>(item)) {
                    scene->removeItem(textItem);
                    delete textItem;
                }
            }
            
            // 정점 삭제
            scene->removeItem(*it);
            delete *it;
            it = vertices.erase(it);
            
            // 남은 정점들의 번호 업데이트
            int newIndex = 0;
            for (auto vertex : vertices) {
                vertex->setToolTip(QString("Vertex %1").arg(newIndex));
                // 라벨 업데이트
                QList<QGraphicsItem*> items = scene->items(vertex->sceneBoundingRect());
                for (QGraphicsItem* item : items) {
                    if (QGraphicsTextItem* textItem = qgraphicsitem_cast<QGraphicsTextItem*>(item)) {
                        textItem->setPlainText(QString::number(newIndex));
                    }
                }
                newIndex++;
            }
            return;
        } else {
            ++it;
        }
    }
    
    // 간선 삭제
    for (auto it = edges.begin(); it != edges.end(); ) {
        if ((*it)->contains((*it)->mapFromScene(pos))) {
            scene->removeItem(*it);
            delete *it;
            it = edges.erase(it);
            return;
        } else {
            ++it;
        }
    }
}

void GraphEditor::updateMousePosition(const QPointF &pos) {
    coordLabel->setText(QString("X: %1, Y: %2").arg(pos.x(), 0, 'f', 1).arg(pos.y(), 0, 'f', 1));
}

void GraphEditor::loadGraph() {
    RCLCPP_INFO(node_->get_logger(), "Starting to load graph...");
    
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Load Graph"), "", tr("YAML files (*.yaml)"));
    
    RCLCPP_INFO(node_->get_logger(), "Selected file: %s", fileName.toStdString().c_str());
    
    if (fileName.isEmpty()) return;
    
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open file");
        return;
    }
    
    // 현재 배경 이미지 저장
    QGraphicsPixmapItem* backgroundImage = nullptr;
    for (auto item : scene->items()) {
        if (auto pixmapItem = qgraphicsitem_cast<QGraphicsPixmapItem*>(item)) {
            backgroundImage = pixmapItem;
            scene->removeItem(backgroundImage);
            break;
        }
    }
    
    // 기존 그래프 정리
    for (auto vertex : vertices) {
        // 그룹 먼저 해제
        if (vertex->group()) {
            scene->destroyItemGroup(vertex->group());
        }
        scene->removeItem(vertex);
        delete vertex;
    }
    vertices.clear();
    
    for (auto edge : edges) {
        scene->removeItem(edge);
        delete edge;
    }
    edges.clear();
    
    // YAML 파일 파싱
    QString content = file.readAll();
    QTextStream stream(&content);
    QMap<int, QPointF> vertexPositions;
    QMap<int, QList<int>> vertexConnections;
    
    int currentIndex = -1;
    int x = 0, y = 0;
    
    while (!stream.atEnd()) {
        QString line = stream.readLine().trimmed();
                
        if (line == "graph:" || line == "vertices:") {
            continue;
        }
        
        // 정점 인덱스 확인
        QRegularExpression indexPattern(R"(^(\d+):$)");
        auto indexMatch = indexPattern.match(line);
        if (indexMatch.hasMatch()) {
            currentIndex = indexMatch.captured(1).toInt();
            continue;
        }
        
        // x 좌표 확인
        if (line.startsWith("x:")) {
            x = line.mid(2).trimmed().toInt();
            continue;
        }
        
        // y 좌표 확인
        if (line.startsWith("y:")) {
            y = line.trimmed().mid(2).trimmed().toInt();
            if (currentIndex >= 0) {
                vertexPositions[currentIndex] = QPointF(x, y);
            }
            continue;
        }
        
        // connections 확인
        if (line.startsWith("connections:")) {
            QString connectionsStr = line.mid(line.indexOf('[') + 1);
            connectionsStr = connectionsStr.left(connectionsStr.indexOf(']'));
            QList<int> connections;
            for (const QString &num : connectionsStr.split(",")) {
                if (!num.trimmed().isEmpty()) {
                    connections.append(num.trimmed().toInt());
                }
            }
            if (currentIndex >= 0) {
                vertexConnections[currentIndex] = connections;
            }
        }
    }
    
    // 정점 생성
    QList<int> sortedKeys = vertexPositions.keys();
    std::sort(sortedKeys.begin(), sortedKeys.end());
    for (int index : sortedKeys) {
        QPointF pos = vertexPositions[index];
        addVertex(pos);
    }
    
    // 간선 생성
    for (auto i = vertexConnections.keyBegin(); i != vertexConnections.keyEnd(); ++i) {
        int fromIndex = *i;
        for (int toIndex : vertexConnections[fromIndex]) {
            if (fromIndex < toIndex) {  // 각 간선을 한 번만 생성
                QLineF line(vertices[fromIndex]->sceneBoundingRect().center(),
                           vertices[toIndex]->sceneBoundingRect().center());
                edges.append(scene->addLine(line, QPen(Qt::black, 2)));
            }
        }
    }
    
    // 배경 이미지 복원
    if (backgroundImage) {
        // 배경 이미지를 맨 뒤로 보내기
        scene->addItem(backgroundImage);
        backgroundImage->setZValue(-1);  // 배경을 맨 뒤로
        
        // 모든 정점과 간선을 배경 위로 가져오기
        for (auto vertex : vertices) {
            vertex->setZValue(1);
            // 정점의 라벨도 함께 가져오기
            if (vertex->group()) {
                for (QGraphicsItem* item : vertex->group()->childItems()) {
                    item->setZValue(1);
                }
            }
        }
        for (auto edge : edges) {
            edge->setZValue(0);  // 간선은 정점보다 아래에
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Graph loading completed");
    file.close();
}

void GraphEditor::subdivideGraph() {
    bool ok;
    double threshold = QInputDialog::getDouble(this, 
        tr("Set Edge Length Threshold"),
        tr("Maximum edge length (pixels):"),
        100.0,  // 기본값
        1.0,    // 최소값
        1000.0, // 최대값
        1,      // 소수점 자리수
        &ok);

    if (!ok) {
        return;  // 사용자가 취소를 누른 경우
    }

    subdivideEdges(threshold);
}

void GraphEditor::subdivideEdges(double threshold) {
    bool needsSubdivision = true;
    
    while (needsSubdivision) {
        needsSubdivision = false;
        QList<QPair<QPointF, QPair<int, int>>> edgesToSubdivide;
        
        // 모든 간선 검사
        for (int i = 0; i < edges.size(); ++i) {
            QLineF line = edges[i]->line();
            double length = line.length();
            
            if (length > threshold) {
                needsSubdivision = true;
                // 중점 계산
                QPointF midPoint = (line.p1() + line.p2()) / 2;
                
                // 연결된 정점들의 인덱스 찾기
                int startIdx = -1, endIdx = -1;
                for (int j = 0; j < vertices.size(); ++j) {
                    QPointF vertexCenter = vertices[j]->sceneBoundingRect().center();
                    if ((vertexCenter - line.p1()).manhattanLength() < 1.0) startIdx = j;
                    if ((vertexCenter - line.p2()).manhattanLength() < 1.0) endIdx = j;
                }
                
                if (startIdx != -1 && endIdx != -1) {
                    edgesToSubdivide.append({midPoint, {startIdx, endIdx}});
                }
            }
        }
        
        // 새로운 정점과 간선 추가
        for (const auto& subdivision : edgesToSubdivide) {
            QPointF midPoint = subdivision.first;
            int startIdx = subdivision.second.first;
            int endIdx = subdivision.second.second;
            
            // 기존 간선 찾아서 삭제
            for (auto it = edges.begin(); it != edges.end(); ) {
                QLineF line = (*it)->line();
                QPointF p1 = vertices[startIdx]->sceneBoundingRect().center();
                QPointF p2 = vertices[endIdx]->sceneBoundingRect().center();
                
                if (((line.p1() - p1).manhattanLength() < 1.0 && 
                     (line.p2() - p2).manhattanLength() < 1.0) ||
                    ((line.p1() - p2).manhattanLength() < 1.0 && 
                     (line.p2() - p1).manhattanLength() < 1.0)) {
                    scene->removeItem(*it);
                    delete *it;
                    it = edges.erase(it);
                } else {
                    ++it;
                }
            }
            
            // 새로운 정점 추가
            QGraphicsEllipseItem* vertex = scene->addEllipse(-5, -5, 10, 10, 
                                                           QPen(Qt::black), 
                                                           QBrush(Qt::white));
            vertex->setPos(midPoint);
            vertices.append(vertex);
            int newVertexIdx = vertices.size() - 1;
            
            // 새로운 간선 추가
            QLineF line1(vertices[startIdx]->sceneBoundingRect().center(),
                        vertices[newVertexIdx]->sceneBoundingRect().center());
            QLineF line2(vertices[newVertexIdx]->sceneBoundingRect().center(),
                        vertices[endIdx]->sceneBoundingRect().center());
                        
            edges.append(scene->addLine(line1, QPen(Qt::black, 2)));
            edges.append(scene->addLine(line2, QPen(Qt::black, 2)));
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
    GraphEditor editor;
    editor.show();
    
    return app.exec();
}
