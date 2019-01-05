#include "ColorBasedTargetDetector.h"
#include <stdexcept>
#include <ros/console.h>

/*
NOTES:

- Dilate and erode dramatically decrease blob time
- Can do blobs every few frames to minimize slowdown
- Blur removed to speed up frame time. Can be added back if desired.
*/

bool operator==(cv::KeyPoint const& lhs, cv::KeyPoint const& rhs)
{
    return lhs.pt == rhs.pt
        && lhs.angle == rhs.angle
        && lhs.size == rhs.size
        && lhs.response == rhs.response
        && lhs.octave == rhs.octave
        && lhs.class_id == rhs.class_id;
}

float ColorBasedTargetDetector::calcOffset(float targetDim, float maxFrameDim)
{
	return (float(targetDim * 2) / maxFrameDim) - 1;
}

void ColorBasedTargetDetector::updateTargetCorrelation(std::vector<cv::KeyPoint> detectedBlobs, int64_t currentTime)
{
    trackedTargets.erase(
        std::remove_if(
            trackedTargets.begin(), trackedTargets.end(),
            [](std::shared_ptr<TargetBoundaryInfo> target){
                if (target->targetBounds->area() == 0) {
                    return false;
                }
                double aspect = std::abs(target->targetBounds->width / double(target->targetBounds->height));
                return aspect < 0.6 || aspect > 1.7;
            }),
        trackedTargets.end());
    /*std::vector<std::
    for (std::shared_ptr<TargetBoundaryInfo> target : trackedTargets)
    {
        double aspect = std::abs(target->targetBounds->width / target->targetBounds->height);
        if (aspect < 0.6 || aspect > 1.7) {

        }
    }*/

    //TODO: Add logging
    std::vector<cv::KeyPoint> unpairedBlobs = detectedBlobs;
    std::vector<std::shared_ptr<TargetBoundaryInfo>> unpairedTargets = trackedTargets;
    for (cv::KeyPoint blob : detectedBlobs)
    {
        for (std::shared_ptr<TargetBoundaryInfo> target : unpairedTargets)
        {
            // TODO: Check for overlap with keypoint diameter as well
            if (target->targetBounds->contains(blob.pt))
            {
                target->lastDetectedTime = currentTime;
                unpairedTargets.erase(std::remove(unpairedTargets.begin(), unpairedTargets.end(), target), unpairedTargets.end());

                for (int i = 0; i < unpairedBlobs.size(); i++) {
                    if (unpairedBlobs[i] == blob)
                        unpairedBlobs.erase(std::next(unpairedBlobs.begin(), i));
                }

                break;
            }
        }
    }

    for (std::shared_ptr<TargetBoundaryInfo> unpairedTarget : unpairedTargets)
    {
        if (currentTime - unpairedTarget->lastDetectedTime >= targetPruneTimeThresh)
            trackedTargets.erase(std::remove(trackedTargets.begin(), trackedTargets.end(), unpairedTarget), trackedTargets.end());
    }

    for (cv::KeyPoint unpairedBlob : unpairedBlobs)
    {
        auto newTarget = std::make_shared<TargetBoundaryInfo>();
        newTarget->firstDetectedTime = currentTime;
        newTarget->lastDetectedTime = currentTime;
        newTarget->targetBounds = std::make_shared<cv::Rect>();
        newTarget->targetBounds->x = (int)unpairedBlob.pt.x;
        newTarget->targetBounds->y = (int)unpairedBlob.pt.y;
        newTarget->targetBounds->width = (int)unpairedBlob.size;
        newTarget->targetBounds->height = (int)unpairedBlob.size;

        trackedTargets.push_back(newTarget);
    }
}

ColorBasedTargetDetector::ColorBasedTargetDetector(ColorBackProjectionEngine& backprojEngine, int64_t targetPruneTimeThresh, uint16_t trackingUpdateInterval)
{
    this->targetPruneTimeThresh = targetPruneTimeThresh;
    this->trackingUpdateInterval = trackingUpdateInterval;
    this->backprojEngine = &backprojEngine;

    // TODO: Tune blob params
    cv::SimpleBlobDetector::Params blobParams;
    blobParams.minDistBetweenBlobs = 20;
    blobParams.minThreshold = 20;
    blobParams.maxThreshold = 180;
    blobParams.thresholdStep = 10;
    blobParams.minRepeatability = 1;

    blobParams.filterByColor = true;
    blobParams.blobColor = 255;

    blobParams.filterByArea = true;
    blobParams.minArea = 20;
    blobParams.maxArea = 240000;

    blobParams.filterByInertia = false;
    blobParams.filterByConvexity = false;
    blobParams.filterByCircularity = false;

    blobDetector = cv::SimpleBlobDetector::create(blobParams);
}

void ColorBasedTargetDetector::updateTracking(cv::Mat newFrame, int64_t currentTime, ColorBackProjectionEngine::Params trackingParams, cv::Mat mask)
{
    cv::Mat backproj_frame;
    this->backprojEngine->update(newFrame, backproj_frame, currentTime, trackingParams, mask);
    //cv::erode(backproj_frame, backproj_frame, cv::Mat(), cv::Point(-1, -1), 3);
    //cv::dilate(backproj_frame, backproj_frame, cv::Mat(), cv::Point(-1, -1), 3);
    cv::erode(backproj_frame, backproj_frame, cv::Mat(), cv::Point(-1, -1), 1);
    //int oddSize = trackingParams.blurSize | 1;
    //cv::GaussianBlur(backproj_frame, this->blurredBackprojBuf, cv::Size(oddSize, oddSize), trackingParams.blurSigma);
    
    //cv::threshold(this->blurredBackprojBuf, this->blurredBackprojBuf, trackingParams.toZeroThresh, 0, CV_THRESH_TOZERO);
    
    //imshow("testtt", this->blurredBackprojBuf);

    if (frameNum % trackingUpdateInterval == 0)
    {
        std::vector<cv::KeyPoint> blobKeyPoints;
        blobDetector->detect(backproj_frame, blobKeyPoints);
        updateTargetCorrelation(blobKeyPoints, currentTime);
    }

    for (auto trackedTarget : trackedTargets)
    {
        // TODO: Tune cam shift params
        if (trackedTarget->targetBounds->area() > 0)
            trackedTarget->lastTrackedPose = CamShift(backproj_frame, *trackedTarget->targetBounds.get(),
                cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 5, 1));

        if (frameWidth > 0 && frameHeight > 0)
	    {
		    trackedTarget->xOffset = calcOffset(trackedTarget->targetBounds->x, frameWidth);
		    trackedTarget->yOffset = calcOffset(trackedTarget->targetBounds->y, frameHeight);
	    }
    }
}

std::vector<std::shared_ptr<TargetBoundaryInfo>> ColorBasedTargetDetector::getTrackedTargets()
{
    return trackedTargets;
}

std::shared_ptr<TargetBoundaryInfo> ColorBasedTargetDetector::getPrimaryTarget()
{
    if (trackedTargets.size() <= 0)
    {
        return nullptr;
    }

    return trackedTargets[0];
}

void ColorBasedTargetDetector::setFrameSize(uint16_t width, uint16_t height)
{
	frameWidth = width; 
	frameHeight = height;
}

ColorBasedTargetDetector::~ColorBasedTargetDetector()
{
}
