// src/components/FileListDock.h
#ifndef FILELISTDOCK_H
#define FILELISTDOCK_H

#include <QDockWidget>
#include <QTableWidget>

class FileListDock : public QDockWidget
{
    Q_OBJECT

public:
    FileListDock(QWidget *parent = nullptr);
    void addFile(const QString& filePath);

signals:
    void fileChecked(const QString& filePath, bool checked);

private:
    QTableWidget* tableWidget;
};

#endif // FILELISTDOCK_H