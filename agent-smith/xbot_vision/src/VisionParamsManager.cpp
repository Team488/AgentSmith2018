
#include "VisionParamsManager.h"
#include "opencv2/opencv.hpp"

bool VisionParamsManager::loadParams(const std::string sourceFile, VisionParams& outParams)
{
    cv::FileStorage file = cv::FileStorage(sourceFile, cv::FileStorage::READ);
    if (!file.isOpened())
        return false;

    file["targetHistogram"] >> outParams.targetHistogram;
    file["enableHsvThreshold"] >> outParams.enableHsvThreshold;
    file["threshHMin"] >> outParams.threshHMin;
    file["threshHMax"] >> outParams.threshHMax;
    file["threshSMin"] >> outParams.threshSMin;
    file["threshSMax"] >> outParams.threshSMax;
    file["threshVMin"] >> outParams.threshVMin;
    file["threshVMax"] >> outParams.threshVMax;

    file["blurSize"] >> outParams.blurSize;
    file["blurSigma"] >> outParams.blurSigma;
    file["toZeroThresh"] >> outParams.toZeroThresh;

    file["floorMargin"] >> outParams.floorMargin;

    return true;
}

bool VisionParamsManager::saveParams(const std::string destFile, VisionParams params)
{
    cv::FileStorage file = cv::FileStorage(destFile, cv::FileStorage::WRITE);
    if (!file.isOpened())
        return false;

    file << "targetHistogram" << params.targetHistogram;
    file << "enableHsvThreshold" << params.enableHsvThreshold;
    file << "threshHMin" << params.threshHMin;
    file << "threshHMax" << params.threshHMax;
    file << "threshSMin" << params.threshSMin;
    file << "threshSMax" << params.threshSMax;
    file << "threshVMin" << params.threshVMin;
    file << "threshVMax" << params.threshVMax;

    file << "blurSize" << params.blurSize;
    file << "blurSigma" << params.blurSigma;
    file << "toZeroThresh" << params.toZeroThresh;

    file << "floorMargin" << params.floorMargin;

    return true;
}
