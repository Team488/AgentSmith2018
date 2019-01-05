#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

class FpsCounter
{
private:
    uint16_t avgCount;
    uint16_t frameTimePos = 0;
    std::vector<int64> frameTimeHist;
    double lastFps;

public:
    FpsCounter(uint16_t avgCount);
    ~FpsCounter();

    void addFrameTimestamp(int64 = 0);
    double getFps();
};

