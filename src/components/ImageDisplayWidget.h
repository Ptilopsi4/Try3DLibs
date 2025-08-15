// src/components/ImageDisplayWidget.h
#ifndef IMAGEDISPLAYWIDGET_H
#define IMAGEDISPLAYWIDGET_H

#include <QWidget>
#include <QLabel>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <QString>

class QVTKOpenGLNativeWidget;

class ImageDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    ImageDisplayWidget(QWidget *parent = nullptr);
    ~ImageDisplayWidget();

    // 加载并显示点云文件
    bool loadPointCloud(const QString& filePath);
    
    // 加载并显示图像
    bool loadImage(const QString& filePath);

private:
    QLabel* imageLabel;
    QVTKOpenGLNativeWidget* vtkWidget;
    vtkSmartPointer<vtkRenderer> renderer;
    
    // 初始化VTK环境
    void initializeVTK();
    
    // 清除当前点云场景
    void clearPointCloudScene();
    
    // 当前模式：0=未显示，1=图像模式，2=点云模式
    int currentMode;
};

#endif // IMAGEDISPLAYWIDGET_H