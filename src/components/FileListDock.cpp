// src/components/FileListDock.cpp
#include "FileListDock.h"
#include <QTableWidget>
#include <QHeaderView>
#include <QCheckBox>
#include <QFileInfo>

FileListDock::FileListDock(QWidget *parent)
    : QDockWidget("File List", parent)
{
    tableWidget = new QTableWidget(this);
    tableWidget->setColumnCount(2);
    tableWidget->setHorizontalHeaderLabels({"File Path", "Load"});
    tableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    tableWidget->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    tableWidget->verticalHeader()->hide();
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);

    setWidget(tableWidget);
}

void FileListDock::addFile(const QString& filePath)
{
    QString absolutePath = QFileInfo(filePath).absoluteFilePath();
    int row = tableWidget->rowCount();
    tableWidget->insertRow(row);
    
    // 文件路径列
    QTableWidgetItem* pathItem = new QTableWidgetItem(absolutePath);
    pathItem->setToolTip(absolutePath);
    pathItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    tableWidget->setItem(row, 0, pathItem);
    
    // 复选框列
    QCheckBox* checkBox = new QCheckBox();
    checkBox->setProperty("filePath", filePath);
    tableWidget->setCellWidget(row, 1, checkBox);

    // 连接信号
    connect(checkBox, &QCheckBox::stateChanged, [this, filePath](int state) {
        emit fileChecked(filePath, state == Qt::Checked);
    });
}