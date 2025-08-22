// src/components/ImageDisplayWidget.cpp
#include "ImageDisplayWidget.h"
#include <QMessageBox>

ImageDisplayWidget::ImageDisplayWidget(QWidget *parent, ElementListDock* elementListDock)
    : QWidget(parent), zoomFactor(1.0), minZoomFactor(0.1), maxZoomFactor(2.0), elementListDock(elementListDock)
{
    setupUI();
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

    grayscaleButton = new QPushButton("Grayscale", this);
    connect(grayscaleButton, &QPushButton::clicked, this, [this]() {
        convertToGrayscale();
    });
    detectShapesButton = new QPushButton("Detect Shapes", this); // 新增按钮
    connect(detectShapesButton, &QPushButton::clicked, this, [this]() {
        detectShapes();
    });

    detectCirclesButton = new QPushButton("Detect Circles", this);
    connect(detectCirclesButton, &QPushButton::clicked, this, [this]() {
        detectCircles();
    });

    zoomInButton = new QPushButton("+", this);
    zoomInButton->setFixedSize(30, 30);
    connect(zoomInButton, &QPushButton::clicked, this, [this]() {
        zoomIn();
    });

    zoomOutButton = new QPushButton("-", this);
    zoomOutButton->setFixedSize(30, 30);
    connect(zoomOutButton, &QPushButton::clicked, this, [this]() {
        zoomOut();
    });

    zoomSlider = new QSlider(Qt::Horizontal, this);
    zoomSlider->setRange(10, 200); // 设置范围
    zoomSlider->setValue(100); // 初始值为100，对应1.0缩放比例
    connect(zoomSlider, &QSlider::valueChanged, this, &ImageDisplayWidget::onZoomSliderValueChanged);

    QHBoxLayout *zoomButtonLayout = new QHBoxLayout();
    zoomButtonLayout->addWidget(zoomOutButton);
    zoomButtonLayout->addWidget(zoomSlider);
    zoomButtonLayout->addWidget(zoomInButton);

    QHBoxLayout *operateButtonLayout = new QHBoxLayout();
    operateButtonLayout->addWidget(grayscaleButton);
    operateButtonLayout->addWidget(detectShapesButton);
    operateButtonLayout->addWidget(detectCirclesButton);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(scrollArea);
    mainLayout->addLayout(zoomButtonLayout);
    mainLayout->addWidget(loadButton);
    mainLayout->addLayout(operateButtonLayout);

    setLayout(mainLayout);
}

vtkSmartPointer<vtkActor> ImageDisplayWidget::createPointCloudActor(const QString& filePath)
{
    QFileInfo fileInfo(filePath);
    std::string stdFilePath = filePath.toStdString();
    
    // 根据文件扩展名选择合适的加载方法
    if (fileInfo.suffix().toLower() == "pcd") {
        // 加载PCD点云文件
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(stdFilePath, *cloud) == -1) {
            // 尝试加载无颜色的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(stdFilePath, *cloudXYZ) == -1) {
                QMessageBox::warning(this, "加载错误", "无法加载PCD文件: " + filePath);
                return nullptr;
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
            
            return actor;
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
            
            return actor;
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
                return nullptr;
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
            
            return actor;
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
            
            return actor;
        }
    }
    else {
        QMessageBox::warning(this, "格式错误", "不支持的点云文件格式: " + fileInfo.suffix());
        return nullptr;
    }
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
    // 清除所有点云Actor
    for (auto it = pointCloudActors.begin(); it != pointCloudActors.end(); ++it) {
        renderer->RemoveActor(it.value());
    }
    pointCloudActors.clear();
    
    vtkWidget->renderWindow()->Render();
}

bool ImageDisplayWidget::removePointCloud(const QString& filePath)
{
    // 如果点云已加载，则移除它
    if (pointCloudActors.contains(filePath)) {
        renderer->RemoveActor(pointCloudActors[filePath]);
        pointCloudActors.remove(filePath);
        vtkWidget->renderWindow()->Render();
        return true;
    }
    return false;
}

void ImageDisplayWidget::clearDisplay()
{
    // 清除点云场景
    clearPointCloudScene();
    
    // 清除图像
    imageLabel->clear();
    imageLabel->setText("图像和点云显示区域");
    
    // 根据当前模式切换显示
    if (currentMode == 2) { // 当前是点云模式
        vtkWidget->hide();
        imageLabel->show();
    }
    
    // 重置模式
    currentMode = 0;
    
    // 更新VTK渲染窗口
    if (vtkWidget->isVisible()) {
        vtkWidget->renderWindow()->Render();
    }
}

bool ImageDisplayWidget::loadMultiplePointClouds(const QSet<QString>& filePaths)
{
    // 首先清除所有现有的点云
    clearPointCloudScene();
    
    // 切换到点云模式
    if (currentMode != 2) {
        imageLabel->hide();
        vtkWidget->show();
        currentMode = 2;
    }
    
    // 加载所有选中的点云文件
    bool allSuccess = true;
    for (const QString& filePath : filePaths) {
        QFileInfo fileInfo(filePath);
        QString suffix = fileInfo.suffix().toLower();
        
        if (suffix == "pcd" || suffix == "ply") {
            bool success = loadPointCloud(filePath, false); // 不清除已有点云
            if (!success) {
                allSuccess = false;
            }
        }
    }
}

void ImageDisplayWidget::detectShapes()
{
    if (originalImage.empty()) {
        QMessageBox::warning(this, "Warning", "No image loaded!");
        return;
    }
    // 图像预处理
    cv::Mat grayImage;
    cv::cvtColor(originalImage, grayImage, cv::COLOR_BGR2GRAY); // 转为灰度图，便于边缘检测
    cv::Mat blurredImage;
    cv::GaussianBlur(grayImage, blurredImage, cv::Size(5, 5), 0); //  高斯模糊，降噪，提高边缘检测准确性
    cv::Mat edgedImage;
    cv::Canny(blurredImage, edgedImage, 50, 150); // Canny算法（边缘检测）

    // 轮廓检测
    std::vector<std::vector<cv::Point>> contours; // 点集
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edgedImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // 只检测最外层

    cv::Mat resultImage = originalImage.clone();
    for (const auto& contour : contours) {
        // 形状识别
        double epsilon = 0.02 * cv::arcLength(contour, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, epsilon, true);
        // 绘制轮廓并标记形状名称
        if (approx.size() == 3) {
            cv::drawContours(resultImage, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 255, 0), 2);
            cv::putText(resultImage, "Triangle", cv::Point(approx[0].x, approx[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        } else if (approx.size() == 4) {
            cv::drawContours(resultImage, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 0, 255), 2);
            cv::putText(resultImage, "Rectangle", cv::Point(approx[0].x, approx[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        } else if (approx.size() > 4) {
            cv::drawContours(resultImage, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(255, 0, 0), 2);
            cv::putText(resultImage, "Circle", cv::Point(approx[0].x, approx[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        }
    }
    // 显示结果
    QImage qImage(resultImage.data, resultImage.cols, resultImage.rows, resultImage.step, QImage::Format_BGR888);
    currentPixmap = QPixmap::fromImage(qImage);
    displayPixmap();
}

void ImageDisplayWidget::detectCircles()
{
    if (originalImage.empty()) {
        QMessageBox::warning(this, "Warning", "No image loaded!");
        return;
    }

    cv::Mat grayImage;
    cv::cvtColor(originalImage, grayImage, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImage, grayImage, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1, grayImage.rows / 8, 200, 100);

    cv::Mat resultImage = originalImage.clone();
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        int radius = c[2];

        // Draw the circle center
        cv::circle(resultImage, center, 3, cv::Scalar(0, 100, 100), cv::FILLED, cv::LINE_AA);
        // Draw the circle outline
        cv::circle(resultImage, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);

        // Add circle to element list
        QString circleName = QString("Circle %1: Center (%2, %3), Radius %4")
                             .arg(i + 1).arg(c[0]).arg(c[1]).arg(c[2]);
        elementListDock->addCircleElement(circleName); // 直接使用成员变量
    }

    QImage qImage(resultImage.data, resultImage.cols, resultImage.rows, resultImage.step, QImage::Format_BGR888);
    currentPixmap = QPixmap::fromImage(qImage);
    displayPixmap();
}