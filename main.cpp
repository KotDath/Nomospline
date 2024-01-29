#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <iostream>

#include "widget/MainWindow.h"

void myMessageHandler(QtMsgType type, const QMessageLogContext &, const QString & msg)
{
    QString txt;
    switch (type) {
        case QtDebugMsg:
            txt = QString("Debug: %1").arg(msg);
            break;
        case QtWarningMsg:
            txt = QString("Warning: %1").arg(msg);
            break;
        case QtCriticalMsg:
            txt = QString("Critical: %1").arg(msg);
            break;
        case QtFatalMsg:
            txt = QString("Fatal: %1").arg(msg);
            abort();
    }
    QFile outFile("my_log.txt");
    outFile.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream ts(&outFile);
    ts << txt << '\n';
    outFile.close();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //qInstallMessageHandler(myMessageHandler);
    MainWindow window;
    window.resize(1080,720);
    window.show();
    return QApplication::exec();
}
