#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("BlueROV GUI vb0.1");
    gst_init(argc,argv);
    w.show();
    return a.exec();
}
