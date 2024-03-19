#pragma once

#include "RenderObject.h"

class BackgroundRenderer : protected RenderObject {
public:
    BackgroundRenderer();
    void initShaders() override;
    void paint() override;
private:
    const float values[8];
};