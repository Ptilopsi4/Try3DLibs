// src/components/VTKDisplayWidget.cpp
#include "VTKDisplayWidget.h"
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSphereSource.h>
#include <vtkConeSource.h>

#include <QHBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>

#include <vtkConeSource.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

VTKDisplayWidget::VTKDisplayWidget(QWidget* parent)
    : QWidget(parent)
{
    // 创建 QVTKOpenGLNativeWidget
    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(this);

    // 创建 VTK 渲染器
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

    // 创建一个立方体
    vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cubeSource->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // 将立方体添加到渲染器
    renderer->AddActor(actor);
    renderer->SetBackground(0.1, 0.2, 0.4); // 设置背景颜色

    // 创建一个球体
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(0.5); // 设置球体半径
    sphereSource->SetThetaResolution(32); // 设置分辨率
    sphereSource->SetPhiResolution(32);

    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(sphereMapper);
    sphereActor->SetPosition(-1.0, 0.0, 0.0); // 设置球体位置

    // 创建一个圆锥
    vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
    coneSource->SetHeight(1.0); // 设置圆锥高度
    coneSource->SetRadius(0.5); // 设置圆锥底面半径
    coneSource->SetResolution(32);

    vtkSmartPointer<vtkPolyDataMapper> coneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    coneMapper->SetInputConnection(coneSource->GetOutputPort());

    vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();
    coneActor->SetMapper(coneMapper);
    coneActor->SetPosition(1.0, 0.0, 0.0); // 设置圆锥位置

    // 将球体和圆锥添加到渲染器
    renderer->AddActor(sphereActor);
    renderer->AddActor(coneActor);

    // 将渲染器连接到 QVTKOpenGLNativeWidget 的渲染窗口
    vtkWidget->renderWindow()->AddRenderer(renderer);

void VTKDisplayWidget::setupCamera()
{
    renderer->ResetCamera();  // 重置相机位置
    renderer->GetActiveCamera()->SetPosition(30, 30, 30);  // 将相机移远
    renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);  // 设置焦点位置
    renderer->GetActiveCamera()->SetViewUp(0, 1, 0);  // 设置视图方向
    renderer->SetBackground(0.2, 0.3, 0.4);  // 设置背景颜色
    renderWindow->Render();  // 渲染窗口
}

void VTKDisplayWidget::displayCircle(double centerX, double centerY, double radius)
{
    QString circleName = QString("Circle %1: Center (%2, %3), Radius %4")
                         .arg(circleActors.size() + 1).arg(centerX).arg(centerY).arg(radius);

    if (circleActors.contains(circleName)) {
        renderer->RemoveActor(circleActors[circleName]);
    }

    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(centerX, centerY, 0);
    sphereSource->SetRadius(radius);
    sphereSource->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0); // 设置颜色为红色

    renderer->AddActor(actor);
    circleActors[circleName] = actor;
    renderWindow->Render();
}

void VTKDisplayWidget::removeCircle(const QString& circleName)
{
    if (circleActors.contains(circleName)) {
        renderer->RemoveActor(circleActors[circleName]);
        circleActors.remove(circleName);
        renderWindow->Render();
    }
}