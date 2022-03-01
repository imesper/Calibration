#include "mainwindow.h"
#include <QVTKOpenGLNativeWidget.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    MainWindow w;
    w.show();
    return a.exec();
}
