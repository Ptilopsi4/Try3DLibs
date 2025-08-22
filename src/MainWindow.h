// src/MainWindow.h
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QSet>
#include <QString>
#include <QFileInfo>

class ElementListDock;
class FileListDock;
class PropertyDisplayDock;
class OperationButtonDock;
class ImageDisplayWidget;
class VTKDisplayWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openFile();
    void handleFileChecked(const QString& filePath, bool checked);
    void handleElementChecked(int index, bool checked);

private:
    ElementListDock* elementListDock;
    FileListDock* fileListDock;
    ImageDisplayWidget* imageDisplayWidget;
    VTKDisplayWidget* vtkDisplayWidget;
    PropertyDisplayDock* propertyDisplayDock;
    OperationButtonDock* operationButtonDock;
    QSet<QString> checkedFiles;
    QString currentDisplayedFile;

    void setupMenu();
    void setupToolBar();
    void createDocks();
    void createCentralWidget();
    void updateDisplayWithCheckedFiles();
};

#endif // MAINWINDOW_H