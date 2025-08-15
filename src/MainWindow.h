// src/MainWindow.h
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class ElementListDock;
class FileListDock;
class ImageDisplayWidget;
class VTKDisplayWidget;
class PropertyDisplayDock;
class OperationButtonDock;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openFile();  
    void handleFileChecked(const QString& filePath, bool checked);

private:
    ElementListDock* elementListDock;
    FileListDock* fileListDock;
    ImageDisplayWidget* imageDisplayWidget;
    VTKDisplayWidget* vtkDisplayWidget;
    PropertyDisplayDock* propertyDisplayDock;
    OperationButtonDock* operationButtonDock;

    void setupMenu();
    void setupToolBar();
    void createDocks();
    void createCentralWidget();
};

#endif // MAINWINDOW_H