// src/components/ImageDisplayWidget.h
#ifndef IMAGEDISPLAYWIDGET_H
#define IMAGEDISPLAYWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QMap>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <QString>

class QVTKOpenGLNativeWidget;

class ImageDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    ImageDisplayWidget(QWidget *parent = nullptr);
    ~ImageDisplayWidget();

    // 加载并显示点云文件
    bool loadPointCloud(const QString& filePath, bool clearExisting = true);
    
    // 加载并显示图像
    bool loadImage(const QString& filePath);
    
    // 清除当前显示内容并重置显示状态
    void clearDisplay();
    
    // 移除特定的点云文件
    bool removePointCloud(const QString& filePath);

    // 加载多个点云文件
    bool loadMultiplePointClouds(const QSet<QString>& filePaths);

private:
    QLabel* imageLabel;
    QVTKOpenGLNativeWidget* vtkWidget;
    vtkSmartPointer<vtkRenderer> renderer;
    
    // 文件路径到Actor的映射，用于管理多个点云
    QMap<QString, vtkSmartPointer<vtkActor>> pointCloudActors;
    
    // 初始化VTK环境
    void initializeVTK();
    
    // 清除当前点云场景
    void clearPointCloudScene();
    
    // 创建点云的Actor
    vtkSmartPointer<vtkActor> createPointCloudActor(const QString& filePath);
    
    // 当前模式：0=未显示，1=图像模式，2=点云模式
    int currentMode;
};

#endif // IMAGEDISPLAYWIDGET_H