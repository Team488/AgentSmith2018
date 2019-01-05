#include <opencv2/opencv.hpp>

class ColorBackProjectionEngine
{
private:
    static const int histogramChannels[];
    /*static const float hueRange[];
    static const float satRange[];
    static const float valRange[];*/
    static const float lRange[];
    static const float aRange[];
    static const float bRange[];
    static const float* const histogramRanges[];
    /*static const int numHueBins;
    static const int numSatBins;
    static const int numValBins;*/
    static const int numLBins;
    static const int numABins;
    static const int numBBins;
    static const int histogramNumBins[];

    cv::Mat rawHistogram;
    cv::Mat targetHistogram;
    cv::Mat backprojFrame;
public:
    struct Params
    {
        int blurSize = 31;
        int blurSigma = 15;
        int toZeroThresh = 10;
        Params() {}
    };

    void updateHistFromTarget(cv::Mat targetImage, cv::Mat targetMask = cv::Mat());
    void generateHistFromRange(cv::Rect2d abRange);
    void getTargetHist(cv::Mat& outHist);
    void setTargetHist(cv::Mat& hist);

    bool hasTargetTraining();
    void displayHistogram();

    void clear();
    void update(cv::Mat& new_hsv_frame, cv::Mat& out, int64_t currentTime, Params trackingParams, cv::Mat mask = cv::Mat());

    void getLastFrame(cv::Mat& out);
};