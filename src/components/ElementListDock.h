#ifndef ELEMENTLISTDOCK_H
#define ELEMENTLISTDOCK_H

#include <QDockWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QCheckBox>
#include <QVector>

class ElementListDock : public QDockWidget
{
    Q_OBJECT

public:
    ElementListDock(QWidget *parent = nullptr);
    QTreeWidget* getTreeWidget() {return treeWidget;}
    void addCircleElement(const QString& name, bool checked = false);

signals:
    void elementChecked(int index, bool checked); // 通知其他组件元素的选中状态变化

private:
    QTreeWidget* treeWidget;
};

#endif // ELEMENTLISTDOCK_H