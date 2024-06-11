#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <iostream>
#include <QtQml>
#include <QUrl>
#include <QQuickWindow>
#include <QFile>

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

void testFileRead() {
    QFile file("C:\\code\\Nomospline\\examples\\BREP\\cylinder\\cylinder.json");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << file.errorString();
    }

    QTextStream in(&file);
    auto val = file.readAll();
    file.close();

    qDebug() << QString(val);
}

int main(int argc, char *argv[])
{
    testFileRead();
    QApplication a(argc, argv);
    QFont fon("Comic Sans", 16);
    a.setFont(fon);
    QQuickWindow::setGraphicsApi(QSGRendererInterface::OpenGL);
    registerTypes();
    QQmlApplicationEngine engine("C:\\code\\Nomospline\\qml\\main.qml");

    return QApplication::exec();
}
