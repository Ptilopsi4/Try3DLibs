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
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
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
    elementListDock = new ElementListDock(this);
    addDockWidget(Qt::LeftDockWidgetArea, elementListDock);

    fileListDock = new FileListDock(this);
    addDockWidget(Qt::LeftDockWidgetArea, fileListDock);

    propertyDisplayDock = new PropertyDisplayDock(this);
    addDockWidget(Qt::BottomDockWidgetArea, propertyDisplayDock);

    operationButtonDock = new OperationButtonDock(this);
    addDockWidget(Qt::BottomDockWidgetArea, operationButtonDock);

    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);

    connect(fileListDock, &FileListDock::fileChecked, this, &MainWindow::handleFileChecked);
}

void MainWindow::createCentralWidget()
{
    imageDisplayWidget = new ImageDisplayWidget(this);
    vtkDisplayWidget = new VTKDisplayWidget(this);

    QSplitter* centralSplitter = new QSplitter(Qt::Horizontal, this);
    centralSplitter->addWidget(imageDisplayWidget);
    centralSplitter->addWidget(vtkDisplayWidget);

    centralSplitter->setSizes({500, 500});

    setCentralWidget(centralSplitter);
}

void MainWindow::openFile()
{
    QStringList filePaths = QFileDialog::getOpenFileNames(
        this,
        "Open Point Cloud",
        "",
        "Point Cloud Files (*.pcd *.ply);;Image Files (*.jpg *.png)"
    );
    
    if (!filePaths.isEmpty()) {
        // 添加所有选择的文件到文件列表
        for (const QString& filePath : filePaths) {
            fileListDock->addFile(filePath);
        }
    }
}

void MainWindow::handleFileChecked(const QString& filePath, bool checked)
{
    QFileInfo fileInfo(filePath);
    QString suffix = fileInfo.suffix().toLower();
    
    if (checked) {
        // 添加到选中文件集合
        checkedFiles.insert(filePath);
        
        // 只处理点云和图像文件
        if (suffix == "pcd" || suffix == "ply") {
            // 如果是点云文件，添加到现有显示中
            imageDisplayWidget->loadPointCloud(filePath, false); // false表示不清除现有点云
        }
        else if (suffix == "jpg" || suffix == "jpeg" || suffix == "png" || suffix == "bmp") {
            // 如果是图像文件，直接显示
            imageDisplayWidget->loadImage(filePath);
        }
        else {
            qDebug() << "不支持的文件类型：" << suffix;
        }
    }
    else {
        // 从选中文件集合中移除
        checkedFiles.remove(filePath);
        
        if (suffix == "pcd" || suffix == "ply") {
            // 如果是点云文件，从显示中移除
            imageDisplayWidget->removePointCloud(filePath);
            
            // 如果没有选中的点云文件了，清空显示
            bool hasPointCloud = false;
            for (const QString& checkedFile : checkedFiles) {
                QFileInfo info(checkedFile);
                if (info.suffix().toLower() == "pcd" || info.suffix().toLower() == "ply") {
                    hasPointCloud = true;
                    break;
                }
            }
            
            if (!hasPointCloud) {
                imageDisplayWidget->clearDisplay();
            }
        }
        else if (suffix == "jpg" || suffix == "jpeg" || suffix == "png" || suffix == "bmp") {
            // 如果是图像文件，需要更新显示
            updateDisplayWithCheckedFiles();
        }
    }
}

void MainWindow::updateDisplayWithCheckedFiles()
{
    // 如果没有选中的文件，则清空显示
    if (checkedFiles.isEmpty()) {
        imageDisplayWidget->clearDisplay();
        return;
    }
    
    // 检查是否有点云文件
    bool hasPointCloud = false;
    QSet<QString> pointCloudFiles;
    
    for (const QString& filePath : checkedFiles) {
        QFileInfo fileInfo(filePath);
        QString suffix = fileInfo.suffix().toLower();
        
        if (suffix == "pcd" || suffix == "ply") {
            hasPointCloud = true;
            pointCloudFiles.insert(filePath);
        }
    }
    
    if (hasPointCloud) {
        // 加载所有选中的点云文件
        imageDisplayWidget->loadMultiplePointClouds(pointCloudFiles);
    }
    else {
        // 没有点云文件，尝试加载图像文件
        for (const QString& filePath : checkedFiles) {
            QFileInfo fileInfo(filePath);
            QString suffix = fileInfo.suffix().toLower();
            
            if (suffix == "jpg" || suffix == "jpeg" || suffix == "png" || suffix == "bmp") {
                imageDisplayWidget->loadImage(filePath);
                break; // 只加载第一个图像文件
            }
        }
    }
}