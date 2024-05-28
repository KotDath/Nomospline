#include <QFile>
#include <QTextStream>
#include "logger.h"


Logger* Logger::instance = nullptr;

void Logger::deployData(const QString &filepath) {
    QFile file(filepath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&file);

        for (const auto& row : data) {
            out << row << '\n';
        }

        file.close();
    }

    data.clear();
}
