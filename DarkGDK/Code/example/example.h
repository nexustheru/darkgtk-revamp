#include <QtWidgets\QMainWindow>
#include "DarkGDK.h"
#include <iostream>

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

private:
	Ui::example *ui;
};
