#include "ElementListDock.h"

ElementListDock::ElementListDock(QWidget *parent)
    : QDockWidget("Element List", parent)
{
    treeWidget = new QTreeWidget(this);
    treeWidget->setColumnCount(2);
    treeWidget->setHeaderLabels(QStringList() << "Elements" << "Select");
    treeWidget->setColumnWidth(0, 200);
    treeWidget->setColumnWidth(1, 50);
    treeWidget->setSelectionMode(QAbstractItemView::NoSelection);

    connect(treeWidget, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column) {
        if (column == 1) {
            int index = treeWidget->indexOfTopLevelItem(item);
            emit elementChecked(index, item->checkState(1) == Qt::Checked);
        }
    });

    setWidget(treeWidget);
}

void ElementListDock::addCircleElement(const QString& name, bool checked)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(treeWidget);
    item->setText(0, name);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(1, checked ? Qt::Checked : Qt::Unchecked);
}