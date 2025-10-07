#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include <qwidget.h>

#include "ui_counterDlg.h"
#include <QTimer>
#include <QSlider>
#include "timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();
        int value=0;

    public slots:
        void doButton();
        void doCount();
        void doReset();
        void doSlider(int);

    private:
        QTimer timer;
        Timer myTimer;


};

#endif // ejemplo1_H
