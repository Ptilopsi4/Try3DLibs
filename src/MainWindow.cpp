// src/MainWindow.cpp
#include "MainWindow.h"
#include "components/ElementListDock.h"
#include "components/FileListDock.h"
#include "components/ImageDisplayWidget.h"
#include "components/VTKDisplayWidget.h"
#include "components/PropertyDisplayDock.h"
#include "components/OperationButtonDock.h"

#include <QSplitter>
#include <QMenuBar>
#include <QToolBar>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // 加载QSS文件
    QFile styleFile(":res/lightStyle.qss");
    styleFile.open(QFile::ReadOnly);
    QString styleSheet = QLatin1String(styleFile.readAll());
    setStyleSheet(styleSheet);

    setGeometry(150, 75, 1400, 900);
    setupMenu();
    setupToolBar();
    createDocks();
    createCentralWidget();
}

MainWindow::~MainWindow() {}

void MainWindow::setupMenu()
{
    QMenuBar* menuBar = new QMenuBar(this);
    setMenuBar(menuBar);

    QMenu* fileMenu = new QMenu("File", this);
    menuBar->addMenu(fileMenu);

    QAction* openAction = new QAction("Open", this);
    QAction* saveAction = new QAction("Save", this);

    fileMenu->addAction(openAction);
    fileMenu->addAction(saveAction);
    
    openAction->setShortcut(QKeySequence("Ctrl+O"));
    saveAction->setShortcut(QKeySequence("Ctrl+S"));

    connect(openAction, &QAction::triggered, this, &MainWindow::openFile);
}

void MainWindow::setupToolBar()
{
    QToolBar* toolBar = new QToolBar("Toolbar", this);
    toolBar->setMovable(true);
    addToolBar(toolBar);

    toolBar->addAction(new QAction("ToolA", this));
    toolBar->addAction(new QAction("ToolB", this));
}

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

    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);

    connect(fileListDock, &FileListDock::fileChecked, this, &MainWindow::handleFileChecked);
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

    // 设置初始大小
    centralSplitter->setSizes({500, 500});

    setCentralWidget(centralSplitter);
}

void MainWindow::openFile()
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "Open Point Cloud",
        "",
        "Point Cloud Files (*.pcd *.ply *.obj *.stl)"
    );
    
    if (!filePath.isEmpty()) {
        // 添加到文件列表
        fileListDock->addFile(filePath);
    }
}

void MainWindow::handleFileChecked(const QString& filePath, bool checked)
{
    if (checked) {
        vtkDisplayWidget->displayPointCloud(filePath);
    } else {
        vtkDisplayWidget->removePointCloud(filePath);
    }
}