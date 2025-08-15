// src/components/ImageDisplayWidget.cpp
#include "ImageDisplayWidget.h"
#include <QVBoxLayout>
#include <QFileInfo>
#include <QMessageBox>
#include <QVTKOpenGLNativeWidget.h>

// VTK 头文件
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkProperty.h>
#include <vtkNamedColors.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>

// PCL 头文件
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>

ImageDisplayWidget::ImageDisplayWidget(QWidget *parent)
    : QWidget(parent), currentMode(0)
{
    // 创建布局
    QVBoxLayout* layout = new QVBoxLayout(this);
    
    // 创建图像标签
    imageLabel = new QLabel(this);
    imageLabel->setAlignment(Qt::AlignCenter);
    imageLabel->setText("图像和点云显示区域");
    imageLabel->setMinimumSize(640, 480);
    
    // 创建VTK部件（初始隐藏)
    vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setMinimumSize(640, 480);
    vtkWidget->hide();
    
    // 初始化VTK环境
    initializeVTK();
    
    // 添加到布局
    layout->addWidget(imageLabel);
    layout->addWidget(vtkWidget);
    
    setLayout(layout);
    setMinimumSize(640, 480);
}

ImageDisplayWidget::~ImageDisplayWidget()
{
    // 清理资源
}

void ImageDisplayWidget::initializeVTK()
{
    // 创建VTK渲染器
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.1, 0.2, 0.4); // 设置背景颜色
    
    // 创建坐标轴
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(0.5, 0.5, 0.5); // 坐标轴长度
    axes->SetShaftType(0);
    axes->SetAxisLabels(1);
    axes->SetCylinderRadius(0.01); // 坐标轴粗细
    renderer->AddActor(axes);
    
    // 配置相机
    renderer->ResetCamera();
    
    // 将渲染器连接到VTK部件
    vtkWidget->renderWindow()->AddRenderer(renderer);
    
    // 设置交互样式
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
        vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    vtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(style);
}

bool ImageDisplayWidget::loadPointCloud(const QString& filePath)
{
    QFileInfo fileInfo(filePath);
    std::string stdFilePath = filePath.toStdString();
    
    // 清除当前点云场景
    clearPointCloudScene();
    
    // 切换到点云模式
    imageLabel->hide();
    vtkWidget->show();
    currentMode = 2;
    
    // 根据文件扩展名选择合适的加载方法
    if (fileInfo.suffix().toLower() == "pcd") {
        // 加载PCD点云文件
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(stdFilePath, *cloud) == -1) {
            // 尝试加载无颜色的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(stdFilePath, *cloudXYZ) == -1) {
                QMessageBox::warning(this, "加载错误", "无法加载PCD文件: " + filePath);
                return false;
            }
            
            // 转换为VTK数据
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            
            // 从PCL点云创建VTK点
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            for (size_t i = 0; i < cloudXYZ->points.size(); ++i) {
                points->InsertNextPoint(cloudXYZ->points[i].x, cloudXYZ->points[i].y, cloudXYZ->points[i].z);
            }
            
            // 创建顶点
            vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
            for (size_t i = 0; i < cloudXYZ->points.size(); ++i) {
                vtkIdType pid[1];
                pid[0] = i;
                vertices->InsertNextCell(1, pid);
            }
            
            // 设置点和顶点
            polydata->SetPoints(points);
            polydata->SetVerts(vertices);
            
            // 创建映射和演员
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polydata);
            
            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetPointSize(2);
            actor->GetProperty()->SetColor(1.0, 1.0, 1.0); // 白色点
            
            // 添加到渲染器
            renderer->AddActor(actor);
        }
        else {
            // 点云有颜色信息，创建VTK数据
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            
            // 从PCL点云创建VTK点
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            colors->SetNumberOfComponents(3);
            colors->SetName("Colors");
            
            for (size_t i = 0; i < cloud->points.size(); ++i) {
                // 添加点
                points->InsertNextPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                
                // 添加颜色
                unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
                colors->InsertNextTypedTuple(color);
            }
            
            // 创建顶点
            vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
            for (size_t i = 0; i < cloud->points.size(); ++i) {
                vtkIdType pid[1];
                pid[0] = i;
                vertices->InsertNextCell(1, pid);
            }
            
            // 设置点、顶点和颜色
            polydata->SetPoints(points);
            polydata->SetVerts(vertices);
            polydata->GetPointData()->SetScalars(colors);
            
            // 创建映射和演员
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polydata);
            
            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetPointSize(2);
            
            // 添加到渲染器
            renderer->AddActor(actor);
        }
    }
    else if (fileInfo.suffix().toLower() == "ply") {
        // 加载PLY点云文件
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(stdFilePath, *cloud) == -1) {
            // 尝试加载无颜色的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPLYFile<pcl::PointXYZ>(stdFilePath, *cloudXYZ) == -1) {
                QMessageBox::warning(this, "加载错误", "无法加载PLY文件: " + filePath);
                return false;
            }
            
            // 转换为VTK数据（与PCD处理类似)
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            
            // 从PCL点云创建VTK点
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            for (size_t i = 0; i < cloudXYZ->points.size(); ++i) {
                points->InsertNextPoint(cloudXYZ->points[i].x, cloudXYZ->points[i].y, cloudXYZ->points[i].z);
            }
            
            // 创建顶点
            vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
            for (size_t i = 0; i < cloudXYZ->points.size(); ++i) {
                vtkIdType pid[1];
                pid[0] = i;
                vertices->InsertNextCell(1, pid);
            }
            
            // 设置点和顶点
            polydata->SetPoints(points);
            polydata->SetVerts(vertices);
            
            // 创建映射和演员
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polydata);
            
            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetPointSize(2);
            actor->GetProperty()->SetColor(1.0, 1.0, 1.0); // 白色点
            
            // 添加到渲染器
            renderer->AddActor(actor);
        }
        else {
            // 处理有颜色的PLY点云（与PCD处理类似)
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            
            // 从PCL点云创建VTK点
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            colors->SetNumberOfComponents(3);
            colors->SetName("Colors");
            
            for (size_t i = 0; i < cloud->points.size(); ++i) {
                // 添加点
                points->InsertNextPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                
                // 添加颜色
                unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
                colors->InsertNextTypedTuple(color);
            }
            
            // 创建顶点
            vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
            for (size_t i = 0; i < cloud->points.size(); ++i) {
                vtkIdType pid[1];
                pid[0] = i;
                vertices->InsertNextCell(1, pid);
            }
            
            // 设置点、顶点和颜色
            polydata->SetPoints(points);
            polydata->SetVerts(vertices);
            polydata->GetPointData()->SetScalars(colors);
            
            // 创建映射和演员
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polydata);
            
            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetPointSize(2);
            
            // 添加到渲染器
            renderer->AddActor(actor);
        }
    }
    else {
        QMessageBox::warning(this, "格式错误", "不支持的点云文件格式: " + fileInfo.suffix());
        return false;
    }
    
    // 重置相机位置以适应点云
    renderer->ResetCamera();
    vtkWidget->renderWindow()->Render();
    
    return true;
}

bool ImageDisplayWidget::loadImage(const QString& filePath)
{
    QImage image(filePath);
    if (image.isNull()) {
        QMessageBox::warning(this, "加载错误", "无法加载图像文件: " + filePath);
        return false;
    }
    
    // 切换到图像模式
    vtkWidget->hide();
    imageLabel->show();
    currentMode = 1;
    
    // 调整图像大小以适应标签，保持纵横比
    QPixmap pixmap = QPixmap::fromImage(image);
    imageLabel->setPixmap(pixmap.scaled(imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    
    return true;
}

void ImageDisplayWidget::clearPointCloudScene()
{
    // 保留坐标轴，移除所有其他演员
    vtkActorCollection* actors = renderer->GetActors();
    actors->InitTraversal();
    
    // 首先收集要移除的演员（避免在遍历过程中修改集合)
    std::vector<vtkActor*> actorsToRemove;
    for (int i = 0; i < actors->GetNumberOfItems(); i++) {
        vtkActor* actor = actors->GetNextActor();
        // 跳过坐标轴（假设坐标轴是第一个添加的演员）
        if (i > 0) {  
            actorsToRemove.push_back(actor);
        }
    }
    
    // 移除收集到的演员
    for (vtkActor* actor : actorsToRemove) {
        renderer->RemoveActor(actor);
    }
}