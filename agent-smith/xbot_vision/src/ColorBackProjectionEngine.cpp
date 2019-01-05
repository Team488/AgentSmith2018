#include "ColorBackProjectionEngine.h"

const int ColorBackProjectionEngine::histogramChannels[] = { 1, 2 };
const float ColorBackProjectionEngine::lRange[] = { 0, 255 };
const float ColorBackProjectionEngine::aRange[] = { 0, 255 };
const float ColorBackProjectionEngine::bRange[] = { 0, 255 };
const float* const ColorBackProjectionEngine::histogramRanges[] = { aRange, bRange };
const int ColorBackProjectionEngine::numLBins = 20;
const int ColorBackProjectionEngine::numABins = 40;
const int ColorBackProjectionEngine::numBBins = 40;
const int ColorBackProjectionEngine::histogramNumBins[] = { numABins, numBBins };

void ColorBackProjectionEngine::updateHistFromTarget(cv::Mat targetImage, cv::Mat targetMask)
{
    cv::Mat new_histogram;
    // TODO: I'm pretty sure that casting a const to a non-const is bad
    cv::calcHist(&targetImage, 1, (int*)histogramChannels, targetMask, new_histogram, 2, (int*)histogramNumBins, (const float**)histogramRanges);
    cv::normalize(new_histogram, new_histogram, 0, 255, cv::NORM_MINMAX);
    cv::namedWindow("Image " + std::to_string((unsigned long long int)targetImage.data), CV_WINDOW_NORMAL);
    cv::imshow("Image " + std::to_string((unsigned long long int)targetImage.data), new_histogram / 255);
    if (rawHistogram.empty()) {
        new_histogram.copyTo(rawHistogram);
    }
    else {
        cv::add(new_histogram, rawHistogram, rawHistogram);
    }
    cv::normalize(rawHistogram, targetHistogram, 0, 255, cv::NORM_MINMAX);
}

void ColorBackProjectionEngine::generateHistFromRange(cv::Rect2d abRange)
{
    targetHistogram = cv::Mat::zeros(2, histogramNumBins, CV_32FC1);
    float aScale = numABins / aRange[1];
    float bScale = numBBins / bRange[1];
    cv::Rect2d newRange(abRange.x * bScale, abRange.y * aScale, abRange.width * bScale, abRange.height * aScale);
    cv::rectangle(targetHistogram, newRange, cv::Scalar(255), CV_FILLED);
}

void ColorBackProjectionEngine::getTargetHist(cv::Mat& outHist)
{
    outHist = targetHistogram;
}

void ColorBackProjectionEngine::setTargetHist(cv::Mat& hist)
{
    targetHistogram = hist;
}

void ColorBackProjectionEngine::clear()
{
    targetHistogram.release();
    rawHistogram.release();
}

bool ColorBackProjectionEngine::hasTargetTraining()
{
    return !targetHistogram.empty();
}

void ColorBackProjectionEngine::displayHistogram()
{
    cv::namedWindow("Target histogram", cv::WINDOW_NORMAL);
    cv::imshow("Target histogram", targetHistogram / 255);
}

void ColorBackProjectionEngine::update(cv::Mat& new_hsv_frame, cv::Mat& out, int64_t currentTime, Params trackingParams, cv::Mat mask)
{
    if (targetHistogram.empty())
        throw std::runtime_error("A histogram must be calculated and loaded before tracking can be executed.");

    cv::calcBackProject(&new_hsv_frame, 1, histogramChannels, targetHistogram, out, (const float**)histogramRanges);
    if (!mask.empty())
        out &= mask;
    
    backprojFrame = out; // HAXX

}

void ColorBackProjectionEngine::getLastFrame(cv::Mat& out)
{
    out = backprojFrame;
}