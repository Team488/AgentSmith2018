#pragma once

#include "ColorBasedTargetDetector.h"
#include "opencv2/opencv.hpp"

struct VisionParams : public ColorBackProjectionEngine::Params
{
    cv::Mat targetHistogram;

    bool enableHsvThreshold = false;
    int threshHMin = 0; int threshHMax = 180;
    int threshSMin = 0; int threshSMax = 255;
    int threshVMin = 0; int threshVMax = 255;
    float floorMargin = 0.1;
};

class VisionParamsManager
{
public:
    static bool loadParams(const std::string sourceFile, VisionParams& outParams);
    static bool saveParams(const std::string destFile, VisionParams params);
};