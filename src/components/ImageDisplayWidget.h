#ifndef IMAGEDISPLAYWIDGET_H
#define IMAGEDISPLAYWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QImage>
#include <QPixmap>
#include <QScrollArea>
#include <QSlider>
#include <opencv2/opencv.hpp>
#include <QComboBox> // 添加下拉菜单

#include "ElementListDock.h"

class ImageDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    ImageDisplayWidget(QWidget *parent = nullptr, ElementListDock* elementListDock = nullptr);

    void loadAndDisplayImage(const QString &filePath);
    void convertToGrayscale();
    void detectShapes();
    void detectCircles();

signals:
    void circleDetected(double centerX, double centerY, double radius);

private slots:
    void zoomIn();
    void zoomOut();
    void onZoomSliderValueChanged(int value);

private:
    QScrollArea *scrollArea;
    QLabel *imageLabel;
    QPushButton *loadButton;
    QPushButton *grayscaleButton;
    QPushButton *zoomInButton;
    QPushButton *zoomOutButton;
    QPushButton *detectShapesButton;
    QPushButton *detectCirclesButton;
    QSlider *zoomSlider;

    cv::Mat originalImage;
    cv::Mat grayscaleImage;
    QPixmap currentPixmap;

    double zoomFactor;
    double minZoomFactor;
    double maxZoomFactor;

    void setupUI();
    void displayPixmap();

    ElementListDock* elementListDock;
};

#endif // IMAGEDISPLAYWIDGET_H