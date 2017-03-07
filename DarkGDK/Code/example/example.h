#include <QtWidgets\QMainWindow>
#include "DarkGDK.h"
#include <iostream>
#include <QtCore\QTimer>



namespace Ui 
{
	class example;
}

class example : public QMainWindow
{
	Q_OBJECT
public:
	explicit example(QWidget *parent = 0);
	~example();

	public slots:
    void update();

private:
	Ui::example *ui;
};

