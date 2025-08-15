// src/components/VTKDisplayWidget.h
#ifndef VTKDISPLAYWIDGET_H
#define VTKDISPLAYWIDGET_H

#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <QMap>

class VTKDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    VTKDisplayWidget(QWidget *parent = nullptr);

    void displayCone();

    void displayPointCloud(const QString& filePath);
    void removePointCloud(const QString& filePath);

private:
    QVTKOpenGLNativeWidget* vtkWidget;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;
    QMap<QString, vtkSmartPointer<vtkActor>> pointCloudActors;  // 键值对存储actor

    void setupCamera();
};

#endif // VTKDISPLAYWIDGET_H