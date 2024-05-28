#pragma once

#include <QString>
#include <QVector>

class Logger {
private:
    static Logger *instance;

    Logger() {}

    Logger(const Logger &);

    Logger &operator=(Logger &);

public:
    static Logger *getInstance() {
        if (!instance)
            instance = new Logger();
        return instance;
    }

private:
    void writeData(float arg) {
        data.last() += QString::number(arg);
    }
    template<typename... Args>
    void writeData(float arg, Args... args)
    {
        data.last() += QString::number(arg) + ' ';
        writeData(args...);
    }
public:
    template<typename... Args>
    void write(float arg, Args... args) {
        data.append("");
        writeData(arg, args...);
    }
    void deployData(const QString& filepath);
private:
    QVector<QString> data;
};