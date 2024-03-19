#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <iostream>
#include <QtQml>
#include <QUrl>

#include "widget/scene.h"
#include "widget/WindowUtils.h"

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

void registerTypes() {
    qmlRegisterType<Scene>("Scene",1,0,"Scene");
    qmlRegisterType<WindowUtils>("WindowUtils",1,0,"WindowUtils");
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QFont fon("Comic Sans", 16);
    a.setFont(fon);

    registerTypes();
    QQmlApplicationEngine engine("../qml/main.qml");

    return QApplication::exec();
}
