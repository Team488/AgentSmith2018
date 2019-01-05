#include "FpsCounter.h"

FpsCounter::FpsCounter(uint16_t avgCount)
{
    frameTimeHist.resize(avgCount);
    this->avgCount = avgCount;
}

FpsCounter::~FpsCounter()
{
}

void FpsCounter::addFrameTimestamp(int64 newTimestamp)
{
    // NOTE: This will give wildly inaccurate readings for the first
    // avgCount frames after the counter is enabled
    uint16_t previousTimePos = frameTimePos;
    frameTimeHist[frameTimePos++] = newTimestamp == 0 ? cv::getTickCount() : newTimestamp;
    if (frameTimePos >= avgCount)
        frameTimePos = 0;

    lastFps = (cv::getTickFrequency() * avgCount) / double(frameTimeHist[previousTimePos] - frameTimeHist[frameTimePos]);
}

double FpsCounter::getFps()
{
    return lastFps;
}

