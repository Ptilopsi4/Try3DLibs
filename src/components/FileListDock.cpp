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
    
    // 复选框列 - 默认选中
    QCheckBox* checkBox = new QCheckBox();
    checkBox->setProperty("filePath", filePath);
    checkBox->setChecked(true); // 设置为选中状态
    tableWidget->setCellWidget(row, 1, checkBox);

    // 连接信号
    connect(checkBox, &QCheckBox::stateChanged, [this, filePath](int state) {
        emit fileChecked(filePath, state == Qt::Checked);
    });
    
    // 由于复选框默认选中，主动发送一次信号通知
    emit fileChecked(filePath, true);
}

void FileListDock::selectFile(const QString& filePath, bool select)
{
    // 遍历表格中的所有行，查找匹配的文件路径
    for (int row = 0; row < tableWidget->rowCount(); ++row) {
        QTableWidgetItem* pathItem = tableWidget->item(row, 0);
        if (pathItem && pathItem->text() == filePath) {
            // 找到匹配的行，设置复选框状态
            QCheckBox* checkBox = qobject_cast<QCheckBox*>(tableWidget->cellWidget(row, 1));
            if (checkBox) {
                checkBox->setChecked(select);
            }
            break;
        }
    }
}