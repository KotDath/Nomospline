#include <QApplication>
#include "widget/MainWindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow window;
    window.resize(1080,720);
    window.show();
    return QApplication::exec();
}
