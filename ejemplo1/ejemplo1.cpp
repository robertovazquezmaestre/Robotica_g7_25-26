#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(resetButton, SIGNAL(clicked()), this, SLOT(doReset()) );
	//connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	myTimer.connect(std::bind(&ejemplo1::doCount,this));
	myTimer.start(500);
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)) );
	timer.start(500);

}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
	if (timer.isActive())
	{
		timer.stop();

	}
	else
	{
		timer.start(500);


	}
	myTimer.stop();

}
void ejemplo1::doCount()
{
	lcdNumber->display(value++);
}

void ejemplo1::doReset()
{
	value=0;
	lcdNumber->display(value);

}

void ejemplo1::doSlider(int f)
{
	int frecuencia = 1000/f;
	timer.setInterval(frecuencia);
	timer.start();
	myTimer.setInterval(frecuencia);
	myTimer.start(frecuencia);
}




