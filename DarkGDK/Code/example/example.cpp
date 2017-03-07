#include <example.h>
#include <ui_example.h>
#include <QtWidgets\qapplication.h>

example::example(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::example)
{
	ui->setupUi(this);
	
}

example::~example()
{
	delete ui;
}

int main(int argc, char *argv[])
{
	dbSyncOn         ( );
	dbSyncRate       ( 60 );
	QApplication a(argc, argv);
	example w;
	//dbPassDLLs();
	if(!LoadSDKDLLs ( ))
		std::cout << "sdkerror";
	if(!CreateSDKApplication(w.width(),w.height(),GetModuleHandle ( NULL ),(HWND)w.winId()))
		std::cout << "apperror";

	dbMakeObjectCube ( 1, 10 );
	while ( LoopGDK ( ) )
	{
		dbRotateObject ( 1, dbObjectAngleX ( 1 ) + 0.1f, dbObjectAngleY ( 1 ) + 0.1f, dbObjectAngleZ ( 1 ) + 0.1f );
		dbSync ( );
	}	
return a.exec();	
}