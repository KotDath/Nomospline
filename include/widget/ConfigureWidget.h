#include <QWidget>
#include "engine/GeometryEngine.h"
#include "widget/OpenGLWindow.h"
#include <QLineEdit>

class ConfigureWidget : public QWidget
{

public:
    ConfigureWidget(OpenGLWindow* parent);
private:
    QLineEdit* normalLength;
    OpenGLWindow* opengl;
};