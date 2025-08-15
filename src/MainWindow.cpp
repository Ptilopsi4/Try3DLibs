// src/MainWindow.cpp
#include "MainWindow.h"
#include "components/ElementListDock.h"
#include "components/FileListDock.h"
#include "components/ImageDisplayWidget.h"
#include "components/VTKDisplayWidget.h"
#include "components/PropertyDisplayDock.h"
#include "components/OperationButtonDock.h"

#include <QSplitter>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setGeometry(150, 75, 1400, 900);
    createDocks();
    createCentralWidget();
}

MainWindow::~MainWindow() {}

void MainWindow::createDocks()
{
    // 创建ElementListDock
    elementListDock = new ElementListDock(this);
    addDockWidget(Qt::LeftDockWidgetArea, elementListDock);

    // 创建FileListDock
    fileListDock = new FileListDock(this);
    addDockWidget(Qt::LeftDockWidgetArea, fileListDock);

    // 创建PropertyDisplayDock
    propertyDisplayDock = new PropertyDisplayDock(this);
    addDockWidget(Qt::BottomDockWidgetArea, propertyDisplayDock);

    // 创建OperationButtonDock
    operationButtonDock = new OperationButtonDock(this);
    addDockWidget(Qt::BottomDockWidgetArea, operationButtonDock);
}

void MainWindow::createCentralWidget()
{
    // 创建ImageDisplayWidget和VTKDisplayWidget
    imageDisplayWidget = new ImageDisplayWidget(this);
    imageDisplayWidget->loadPointCloud("C:/Users/14054/Downloads/wires_boxes/frame_00073.pcd");
    vtkDisplayWidget = new VTKDisplayWidget(this);

    QSplitter* centralSplitter = new QSplitter(Qt::Horizontal, this);
    centralSplitter->addWidget(imageDisplayWidget);
    centralSplitter->addWidget(vtkDisplayWidget);

    setCentralWidget(centralSplitter);
}