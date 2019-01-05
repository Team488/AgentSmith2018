#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>
#include "ColorBackProjectionEngine.h"

struct TargetBoundaryInfo
{
    std::shared_ptr<cv::Rect> targetBounds;
    cv::RotatedRect lastTrackedPose;
    int64 firstDetectedTime = -1;
    int64 lastDetectedTime = -1;
	float xOffset = 0; 
	float yOffset = 0;
};

class ColorBasedTargetDetector
{
private:
    int64_t targetPruneTimeThresh;
    uint16_t trackingUpdateInterval;
	uint64_t frameNum = 0;
	uint16_t maxTargets = 20;
	uint16_t frameWidth = 0;
	uint16_t frameHeight = 0;

    cv::Mat blurredBackprojBuf;

    ColorBackProjectionEngine *backprojEngine;

    // Use shared_ptr to make it easy to dynamically modify struct values
    std::vector<std::shared_ptr<TargetBoundaryInfo>> trackedTargets;
    cv::Ptr<cv::SimpleBlobDetector> blobDetector;
	float_t calcOffset(float_t targetDim, float_t maxFrameDim);

    void updateTargetCorrelation(std::vector<cv::KeyPoint> detectedBlobs, int64_t currentTime);

public:

    ColorBasedTargetDetector(ColorBackProjectionEngine& backprojEngine, int64_t targetPruneTimeThresh, uint16_t trackingUpdateInterval);
    void updateTracking(cv::Mat newFrame, int64_t currentTime, ColorBackProjectionEngine::Params trackingParams, cv::Mat mask = cv::Mat());

    std::vector<std::shared_ptr<TargetBoundaryInfo>> getTrackedTargets();
	void setFrameSize(uint16_t width, uint16_t height);
    std::shared_ptr<TargetBoundaryInfo> getPrimaryTarget();

    ~ColorBasedTargetDetector();
};
