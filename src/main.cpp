// src/main.cpp
#include <QApplication>
#include "mainwindow.h"
#include <QVTKOpenGLNativeWidget.h>
#include <QPointer>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}