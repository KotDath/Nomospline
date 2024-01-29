#include <chrono>
#include <QString>
#include <QDebug>

class Timer
{
public:
    Timer(const QString& startName, const QString& endName);
    ~Timer();
    void Stop();
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_StartTimepoint;
    QString startName;
    QString endName;

    bool isStopped = false;
};
