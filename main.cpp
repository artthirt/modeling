#include "mainwindow.h"
#include <QApplication>

#include <QDir>

int main(int argc, char *argv[])
{
	QString path = argv[0];
	QDir dir(path);
	dir.cd("..");
	path = dir.canonicalPath();

	QDir::setCurrent(path);

	QApplication a(argc, argv);
	MainWindow w;
	w.show();

	return a.exec();
}
