// src/main.cpp
#include <QApplication>
#include <Windows.h>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    HWND hWnd = GetConsoleWindow();  // 获取CMD窗口句柄
    ShowWindow(hWnd, SW_HIDE);       // 隐藏CMD窗口

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}