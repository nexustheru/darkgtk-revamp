#include <example.h>
#include <ui_example.h>
#include <QtWidgets\qapplication.h>

example::example(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::example)
{
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(1000);
	ui->setupUi(this);
	//
	dbSyncOn         ( );
	dbSyncRate       ( 60 );
	if(!LoadSDKDLLs ( ))
		std::cout << "sdkerror";
	if(!CreateSDKApplication(this->width(),this->height(),GetModuleHandle ( NULL ),(HWND)this->winId()))
		std::cout << "apperror";
    dbMakeObjectCube ( 1, 10 );
}

void example::update()
{
	while ( LoopGDK ( ) )
	{
		dbRotateObject ( 1, dbObjectAngleX ( 1 ) + 0.1f, dbObjectAngleY ( 1 ) + 0.1f, dbObjectAngleZ ( 1 ) + 0.1f );
		dbSync ( );
	}	
	
}

example::~example()
{
	delete ui;
}

int main(int argc, char *argv[])
{
	
	QApplication a(argc, argv);
	example w;
return a.exec();	
}
