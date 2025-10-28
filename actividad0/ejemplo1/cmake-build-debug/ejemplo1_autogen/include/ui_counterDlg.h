/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QPushButton *button;
    QLCDNumber *lcdNumber;
    QPushButton *resetButton;
    QSlider *horizontalSlider;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName("Counter");
        Counter->resize(568, 400);
        button = new QPushButton(Counter);
        button->setObjectName("button");
        button->setGeometry(QRect(20, 210, 251, 71));
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName("lcdNumber");
        lcdNumber->setGeometry(QRect(110, 40, 301, 91));
        resetButton = new QPushButton(Counter);
        resetButton->setObjectName("resetButton");
        resetButton->setGeometry(QRect(300, 210, 221, 71));
        horizontalSlider = new QSlider(Counter);
        horizontalSlider->setObjectName("horizontalSlider");
        horizontalSlider->setGeometry(QRect(160, 340, 231, 31));
        horizontalSlider->setOrientation(Qt::Horizontal);

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QCoreApplication::translate("Counter", "Counter", nullptr));
        button->setText(QCoreApplication::translate("Counter", "STOP", nullptr));
        resetButton->setText(QCoreApplication::translate("Counter", "RESET", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
