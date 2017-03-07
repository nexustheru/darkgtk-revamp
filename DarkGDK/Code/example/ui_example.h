/********************************************************************************
** Form generated from reading UI file 'example.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EXAMPLE_H
#define UI_EXAMPLE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_example
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *example)
    {
        if (example->objectName().isEmpty())
            example->setObjectName(QStringLiteral("example"));
        example->resize(400, 300);
        menuBar = new QMenuBar(example);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        example->setMenuBar(menuBar);
        mainToolBar = new QToolBar(example);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        example->addToolBar(mainToolBar);
        centralWidget = new QWidget(example);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        example->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(example);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        example->setStatusBar(statusBar);

        retranslateUi(example);

        QMetaObject::connectSlotsByName(example);
    } // setupUi

    void retranslateUi(QMainWindow *example)
    {
        example->setWindowTitle(QApplication::translate("example", "example", 0));
    } // retranslateUi

};

namespace Ui {
    class example: public Ui_example {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EXAMPLE_H
