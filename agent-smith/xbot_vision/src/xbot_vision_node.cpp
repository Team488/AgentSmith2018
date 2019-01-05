#include <stdio.h>
#include <string.h>
#include <cmath>
#include <memory>
#include <algorithm>
#include <fstream>

#include <sys/stat.h>
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>

#include "VisionParamsManager.h"
#include "FpsCounter.h"
#include "TimeUtils.h"
#include <xbot_vision/DetectedCube.h>
#include <xbot_vision/CubeTargetInfo.h>

// ZED include
#include <sl/Camera.hpp>

#define PI (float(3.1415926535))

void do_CLAHE_output_LAB(cv::Mat& bgr_image_in, cv::Mat& lab_image_out)
{
    cv::cvtColor(bgr_image_in, lab_image_out, CV_BGR2Lab);
/*
    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image_out, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image_out);*/
}

void do_CLAHE(cv::Mat& bgr_image_in, cv::Mat& bgr_image_out)
{
    // TODO: Move out
    cv::Mat lab_image;
    do_CLAHE_output_LAB(bgr_image_in, lab_image);

    // convert back to RGB
    cv::Mat image_clahe;
    cv::cvtColor(lab_image, bgr_image_out, CV_Lab2BGR);
}

class VisionNode
{
  private:
    ros::NodeHandle node;
    ros::Publisher trackedCubePublisher;
    ros::Publisher visualizableTrackedCubePublisher;

    ros::Subscriber is_match_running_subscriber;
    bool* is_match_running_ptr = nullptr;

    tf::TransformListener transform_listener;

    bool is_gui_mode;
    cv::Rect* selectedTarget = nullptr;

    TimerManager timers;

    bool frame_size_set = false;

    std::string config_file_path;
    VisionParams params;

    // 1 second prune time threshold
    const int64 targetPruneTime = (int64)(cv::getTickFrequency() * 1);

    ColorBackProjectionEngine backproj_engine;
    ColorBasedTargetDetector targetDetector = ColorBasedTargetDetector(backproj_engine, this->targetPruneTime, 5);

    cv::Mat hsv_frame;
    cv::Mat thresh_frame;
    cv::Mat clahe_frame_lab;
    cv::Mat backprojFrameBuf;

    static void onMouse(int event, int x, int y, int flags, void* userData)
    {
        cv::Rect** selectedTargetRef = (cv::Rect**)userData;
        if (event == cv::EVENT_LBUTTONDOWN)
        {
            if (*selectedTargetRef != nullptr)
                delete *selectedTargetRef;

            *selectedTargetRef = new cv::Rect(x, y, 0, 0);
        }
        else if (event == cv::EVENT_LBUTTONUP && *selectedTargetRef != nullptr)
        {
            cv::Rect* selectedTarget = *selectedTargetRef;

            if(x > selectedTarget->x)
                selectedTarget->width = x - selectedTarget->x;
            else if (x < selectedTarget->x)
            {
                selectedTarget->width = selectedTarget->x - x;
                selectedTarget->x = x;
            }

            if (y > selectedTarget->y)
                selectedTarget->height = y - selectedTarget->y;
            else if (y < selectedTarget->y)
            {
                selectedTarget->height = selectedTarget->y - y;
                selectedTarget->y = y;
            }
        }
    }

    void updateTrainingFromSelection()
    {
        if (selectedTarget != nullptr && selectedTarget->area() > 0)
        {
            // TODO: Transition selection code to smart_ptr
            //printf("Updating target histogram with rect (x: %d, y: %d) (w: %d, h: %d)\r\n", selectedTarget->x, selectedTarget->y, selectedTarget->width, selectedTarget->height);
            cv::Mat sourceRoi = cv::Mat(clahe_frame_lab, *selectedTarget);
            cv::Mat maskRoi = thresh_frame.empty() ? cv::Mat() : cv::Mat(thresh_frame, *selectedTarget);

            //cv::Mat calculatedHistogram;
            // TODO: Distinguish hue / hue_sat modes
            this->backproj_engine.updateHistFromTarget(sourceRoi, maskRoi);
            this->backproj_engine.getTargetHist(params.targetHistogram);

            // TODO: Re-write histogram render for 2d histogram with sat
            //std::cout << targetColorHistogram << std::endl;

            /*if (excludeSaturationInHist)
            {
                histogramRender = cv::Mat::zeros(200, calculatedHistogram.rows * 10, CV_8UC3);
                this->renderHueHistogram(calculatedHistogram, histogramRender, calculatedHistogram.rows);
                cv::imshow("Histogram render", histogramRender);
            }*/

            delete selectedTarget;
            selectedTarget = nullptr;
        }
    }

    void handle_match_running_update(const std_msgs::Bool& packet)
    {
        if (is_match_running_ptr != nullptr) {
            // Pointer is a hack; camera ingest isn't in a class right now, and there are no appropriate node.subscribe() overloads
            *is_match_running_ptr = packet.data;
        }
    }

  public:
    VisionNode(ros::NodeHandle node, bool* is_match_running_ptr) : node(node)
    {
        // TODO: Necessary?
        backprojFrameBuf.create(720, 1280, CV_8UC1);
        this->is_match_running_ptr = is_match_running_ptr;

        is_match_running_subscriber = node.subscribe("/comms/is_match_running", 1, &VisionNode::handle_match_running_update,this);
    }

    void initialize()
    {
        ros::param::get("/xbot_vision_node/is_gui", is_gui_mode);

        this->trackedCubePublisher = this->node.advertise<xbot_vision::CubeTargetInfo>("/vision/tracked_cube_target", 1, false);
        this->visualizableTrackedCubePublisher = this->node.advertise<geometry_msgs::PointStamped>("/vision/visualizable_tracked_cube_target", 1, false);

        ros::param::get("/xbot_vision_node/config_file", config_file_path);
        VisionParamsManager::loadParams(config_file_path, this->params);
        if (this->params.targetHistogram.empty())
        {
            ROS_WARN_COND(!is_gui_mode, "No target training loaded and not running in GUI mode! Target detection won't operate.");
        }
        else
        {
            this->backproj_engine.setTargetHist(params.targetHistogram);
        }

        if (is_gui_mode) {
            cv::namedWindow("Source", CV_WINDOW_NORMAL);
            cv::namedWindow("CLAHE", CV_WINDOW_NORMAL);
            cv::namedWindow("Backprojected frame", CV_WINDOW_NORMAL);
            cv::namedWindow("Processed frame", CV_WINDOW_NORMAL);
            cv::namedWindow("LAB Frame", CV_WINDOW_NORMAL);

            setMouseCallback("Source", this->onMouse, &selectedTarget);
        }
    }

    sl::float4 get_position_for_target_rect(cv::Rect& bounds, sl::Mat& pointCloud)
    {
        cv::Point targetPoint = (bounds.br() + bounds.tl()) * 0.5;
        sl::float4 point3D;
        pointCloud.getValue(targetPoint.x, targetPoint.y, &point3D);
        return point3D;
    }

    /**
     * Returns the "slope" s of the floor plane relative to the zed, such that s multiplied by a z
     * distance in zed frame results in the theoretical x distance from the zed to the floor at that point.
     */
    double get_floor_plane_slope()
    {
        tf::StampedTransform transform;
        try {
            transform_listener.lookupTransform("/base_zed", "/gripper_middle", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return 0;
        }
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

        return -std::tan(pitch);
    }

    bool find_initial_pixel(cv::Mat& backprojected_frame, cv::Point2i& outPoint)
    {
        for(int y = backprojected_frame.rows - 1; y >= 0; y--)
        {
            if(cv::countNonZero(backprojected_frame.row(y)) > 0)
            {
                for(size_t x = 0; x < backprojected_frame.cols; x++)
                {
                    if (backprojected_frame.at<uchar>(y, x) > params.toZeroThresh)
                    {
                        outPoint = cv::Point2i(x, y);
                        return true;
                    }
                }
            }
        }

        /*int nRows = backprojected_frame.rows;
        int nCols = backprojected_frame.cols;

        int x, y;
        uchar* p;
        for(y = nRows; y >= 0; --y)
        {
            p = backprojected_frame.ptr<uchar>(y);
            for (x = 0; x < nCols; ++x)
            {
                if (p[x] > params.toZeroThresh)
                {
                    outPoint = cv::Point2i(x, y);
                    return true;
                }
            }
        }*/
        
        return false;
    }

    sl::float3 get_spatial_point(cv::Rect rect, sl::Mat& point_cloud)
    {
        double x_sum = 0, y_sum = 0, z_sum = 0;
        sl::float4 point;
        uint64_t count = 0;
        for (int x = rect.x; x < rect.x + rect.width; x++)
        {
            for (int y = rect.y; y < rect.y + rect.height; y++)
            {
                point_cloud.getValue(x, y, &point);

                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
                {
                    count++;
                    x_sum += point.x;
                    y_sum += point.y;
                    z_sum += point.z;
                }
            }
        }

        return sl::float3(x_sum / count, y_sum / count, z_sum / count);
    }

    bool identify_cube_target(cv::Mat& backprojected_frame, sl::Mat& point_cloud, geometry_msgs::PointStamped& out_point)
    {
        cv::Point2i initial_pixel;
        if (find_initial_pixel(backprojected_frame, initial_pixel))
        {
            //cv::GaussianBlur(backprojFrameBuf, backprojFrameBuf, cv::Size(23, 23), 19);
            cv::blur(backprojected_frame, backprojected_frame, cv::Size(6, 6));
            //cv::floodFill(backprojected_frame, initial_pixel, cv::Scalar(255), NULL, cv::Scalar(10), cv::Scalar(255));
            cv::Rect window(initial_pixel.x - 25, initial_pixel.y - 50, 50, 50);
            cv::RotatedRect target = cv::CamShift(backprojected_frame, window,
                cv::TermCriteria(cv::TermCriteria::COUNT, 2, 0));

            if(is_gui_mode)
            {
                cv::rectangle(backprojected_frame, window, cv::Scalar(255));
                if(target.size.width >= 0 && target.size.height >= 0)
                {
                    cv::ellipse(backprojected_frame, target, cv::Scalar(255), 1);
                }
            }

            //cv::imshow("AAA", resizedBackprojFrameBuf);

            /*sl::float4 point_zed_coordinates;
            point_cloud.getValue(target.center.x, target.center.y, &point_zed_coordinates);*/
            sl::float3 point_zed_coordinates = get_spatial_point(target.boundingRect(), point_cloud);
            
            
            //if(std::isnan)
            //sl::float3 point_ros_coordinates(point_zed_coordinates.y, -point_zed_coordinates.x, point_zed_coordinates.z);

            geometry_msgs::PointStamped zed_frame_point;
            zed_frame_point.header.stamp = ros::Time();
            zed_frame_point.header.frame_id = "/base_zed";
            zed_frame_point.point.x = point_zed_coordinates.y;
            zed_frame_point.point.y = -point_zed_coordinates.x;
            zed_frame_point.point.z = point_zed_coordinates.z;
            try {
                transform_listener.transformPoint("/gripper_middle", zed_frame_point, out_point);
            std::cout << zed_frame_point.point.y << "    " << out_point.point.y << std::endl;
                return true;
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                return false;
            }
        }
        return false;
        /*for(size_t y = point_cloud.getHeight() - 1; y >= 0; y--)
        {
            if(cv::countNonZero(backprojected_frame.row(y)) > 0)
            {
                for(size_t x = 0; x < backprojected_frame.cols; x++)
                {
                    if (backprojected_frame.at<uchar>(y, x) > params.toZeroThresh)
                    {
                        cv::circle(backprojected_frame, cv::Point(x, y), 5, cv::Scalar(255), 3);
                        return sl::float3(0, 0, 0);
                    }
                }
            }
        }*/

        /*for(size_t y = point_cloud.getHeight() - 1; y >= 0; y--)
        {
            for(size_t x = 0; x < point_cloud.getWidth(); x++)
            {
                if (backprojected_frame.at<uchar>(y, x) > params.toZeroThresh)
                {
                    cv::circle(backprojected_frame, cv::Point(x, y), 5, cv::Scalar(255), 3);
                    return sl::float3(0, 0, 0);
                }
            }
        }*/

        /*
        tf::StampedTransform transform;
        try {
            transform_listener.lookupTransform("/base_zed", "/robot_footprint", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            //return 0;
        }

        cv::Mat tmp(backprojected_frame.rows, backprojected_frame.cols, CV_8UC1);
        cv::Mat tmp2(backprojected_frame.rows, backprojected_frame.cols, CV_8UC1);
        //double floor_slope = get_floor_plane_slope();
        sl::float4 point_zed_coordinates;


        tf::Point zed_point;
        cv::Point2i min_depth_pixel;
        double min_depth_val = INFINITY;
        for(size_t x = 0; x < point_cloud.getWidth(); x++)
        {
            for(size_t y = 0; y < point_cloud.getHeight(); y++)
            {
                if(backprojected_frame.at<uchar>(y, x) <= params.toZeroThresh)
                {
                    backprojected_frame.at<uchar>(y, x) = 0;
                    tmp.at<uchar>(y, x) = 255;
                    continue;
                }
                point_cloud.getValue(x, y, &point_zed_coordinates);
                //sl::float3 point_ros_coordinates(point_zed_coordinates.y, -point_zed_coordinates.x, point_zed_coordinates.z);
                //double floor_dist = floor_slope * point_ros_coordinates.z;
                //ROS_ERROR_STREAM("Dist " << floor_dist);
                zed_point.setX(point_zed_coordinates.y);
                zed_point.setY(-point_zed_coordinates.x);
                zed_point.setZ(point_zed_coordinates.z);

                tf::Point floor_plane_point = transform * zed_point;

                /*geometry_msgs::PointStamped floor_plane_point;
                try {
                    //transform_listener.transformPoint("/robot_footprint", zed_point, floor_plane_point);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    //return;
                }* /
                //tmp.at<uchar>(y, x) = (unsigned char)(point_ros_coordinates.x / 10 * 255);
                tmp2.at<uchar>(y, x) = (unsigned char)(floor_plane_point.z()  / 2 * 255);
                if (floor_plane_point.z() > params.floorMargin)//point_ros_coordinates.x < floor_dist - params.floorMargin && backprojected_frame.at<uchar>(y, x) > params.toZeroThresh)
                {
                    // TODO
                }
                else
                {
                    backprojected_frame.at<uchar>(y, x) = 0;
                    tmp.at<uchar>(y, x) = 255;
                }
            }
        }
        imshow("Depth", tmp);
        imshow("Debug", tmp2);*/
    }

    void process_frame_simplified_algorithm(uint32_t frameNumber, cv::Mat newColorFrame, sl::Mat newPointCloud)
    {
        cv::Mat clahe_frame; // TODO: Global?
        do_CLAHE(newColorFrame, clahe_frame);
        if(is_gui_mode) {
            imshow("CLAHE", clahe_frame);
        }
        cvtColor(clahe_frame, hsv_frame, CV_BGR2HSV);

        //cv::medianBlur(hsv_frame, hsv_frame, 13);

        /*cv::Mat bgrOut;
        cvtColor(hsv_frame, bgrOut, CV_HSV2BGR);

        imshow("AAA", bgrOut);*/
        if (is_gui_mode) {
            updateTrainingFromSelection();
        }

        if (backproj_engine.hasTargetTraining())
        {
            this->backproj_engine.update(hsv_frame, backprojFrameBuf, frameNumber, this->params);
            if(is_gui_mode)
            {
                cv::imshow("Backprojected frame", backprojFrameBuf);
            }

            //cv::GaussianBlur(backprojFrameBuf, backprojFrameBuf, cv::Size(11, 11), 5);
            //cv::medianBlur(backprojFrameBuf, backprojFrameBuf, 13);
            //cv::threshold(backprojFrameBuf, backprojFrameBuf, params.toZeroThresh, 0, CV_THRESH_TOZERO);
            cv::erode(backprojFrameBuf, backprojFrameBuf, cv::Mat(), cv::Point(-1, -1), 1);
            //cv::dilate(backprojFrameBuf, backprojFrameBuf, cv::Mat(), cv::Point(-1, -1), 3);
            
            xbot_vision::CubeTargetInfo targetInfo;

            geometry_msgs::PointStamped cube_target;
            if (identify_cube_target(backprojFrameBuf, newPointCloud, cube_target))
            {
                targetInfo.has_tracked_cube = true;
                targetInfo.target_cube_point = cube_target;
            }
            else
            {
                targetInfo.has_tracked_cube = false;
                targetInfo.target_cube_point.header.frame_id = "/gripper_middle";
                targetInfo.target_cube_point.point.x = 0;
                targetInfo.target_cube_point.point.y = 0;
                targetInfo.target_cube_point.point.z = 0;
            }
            trackedCubePublisher.publish(targetInfo);
            visualizableTrackedCubePublisher.publish(targetInfo.target_cube_point);
            if(is_gui_mode)
            {
                cv::imshow("Processed frame", backprojFrameBuf);
            }
        }

        if (is_gui_mode) {
            cv::imshow("Source", newColorFrame);
        }
    }

    void do_wait_key(cv::Mat colorImage)
    {
        if (is_gui_mode)
        {
            char key = cv::waitKey(1);
            if (key == 's') {
                VisionParamsManager::saveParams(config_file_path, this->params);
                std::cout << "Saved params to " << config_file_path << std::endl;
            }
            else if (key == 'c') {
                backproj_engine.clear();
                std::cout << "Cleared histogram" << std::endl;
            }
            else if (key == 'i') {
                time_t seconds = time(NULL);
                std::string file_name = ros::package::getPath("xbot_vision") + "/../xbot_data/cube_training/img_" + std::to_string(seconds) + ".png";
                cv::imwrite(file_name, colorImage);
                std::cout << "Captured to " << file_name << std::endl;
            }
        }
    }

    void process_frame_blob_and_shift_algorithm(uint32_t frameNumber, cv::Mat newColorFrame, sl::Mat newPointCloud)
    {
        if (!frame_size_set)
        {
            frame_size_set = true;
            this->targetDetector.setFrameSize(newColorFrame.cols, newColorFrame.rows);
        }

        //medianBlur(sourceFrame, sourceFrame, 5);

        do_CLAHE_output_LAB(newColorFrame, clahe_frame_lab);

        if (is_gui_mode) {
            updateTrainingFromSelection();
        }

        if (backproj_engine.hasTargetTraining())
        {
            if (is_gui_mode) {
                backproj_engine.displayHistogram();
                cv::imshow("LAB Frame", clahe_frame_lab);
            }

            uint64_t now = cv::getTickCount();
            targetDetector.updateTracking(clahe_frame_lab, now, params);
            //ROS_INFO_STREAM("NUM " << targets.size());

            auto allTargetBoundaries = targetDetector.getTrackedTargets();
            std::vector<std::tuple<std::shared_ptr<TargetBoundaryInfo>, geometry_msgs::PointStamped>> allTargets;

            for (auto target : allTargetBoundaries)
            {
                sl::float3 point_zed_coordinates = get_spatial_point(target->lastTrackedPose.boundingRect(), newPointCloud);

                geometry_msgs::PointStamped robot_point;

                geometry_msgs::PointStamped zed_frame_point;
                zed_frame_point.header.stamp = ros::Time();
                zed_frame_point.header.frame_id = "/base_zed";
                zed_frame_point.point.x = point_zed_coordinates.y;
                zed_frame_point.point.y = -point_zed_coordinates.x;
                zed_frame_point.point.z = point_zed_coordinates.z;
                try {
                    transform_listener.transformPoint("/gripper_middle", zed_frame_point, robot_point);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    continue;
                }

                auto tuple = std::make_tuple(target, robot_point);
                allTargets.push_back(tuple);
            }

            std::vector<std::tuple<std::shared_ptr<TargetBoundaryInfo>, geometry_msgs::PointStamped>> possibleTargets;

            std::copy_if (allTargets.begin(), allTargets.end(), std::back_inserter(possibleTargets), [&now](std::tuple<std::shared_ptr<TargetBoundaryInfo>, geometry_msgs::PointStamped> targetInfo){
                    double secondsSinceLastDetection = (now - std::get<0>(targetInfo)->lastDetectedTime) / double(cv::getTickFrequency());
                    //std::cout << secondsSinceLastDetection << " :::: " << std::get<1>(targetInfo).point.z << std::endl;
                    double aspect = std::abs(std::get<0>(targetInfo)->targetBounds->width / double(std::get<0>(targetInfo)->targetBounds->height));
                    return //(now - std::get<0>(targetInfo)->firstDetectedTime) / cv::getTickFrequency() > 1
                        aspect > 0.6 && aspect < 1.7
                        && secondsSinceLastDetection < 0.4
                        && std::get<1>(targetInfo).point.z < 0.3;
                });
            
            //std::cout << possibleTargets.size() << std::endl;
            bool hasTarget = possibleTargets.size() >= 1;
            std::tuple<std::shared_ptr<TargetBoundaryInfo>, geometry_msgs::PointStamped> closest;
            
            if (hasTarget) {
                closest = *std::min_element(possibleTargets.begin(), possibleTargets.end(), [](std::tuple<std::shared_ptr<TargetBoundaryInfo>, geometry_msgs::PointStamped>& a, std::tuple<std::shared_ptr<TargetBoundaryInfo>, geometry_msgs::PointStamped>& b) {
                    auto a_point = std::get<1>(a).point;
                    auto b_point = std::get<1>(b).point;
                    return std::sqrt(a_point.x * a_point.x + a_point.y * a_point.y) < std::sqrt(b_point.x * b_point.x + b_point.y * b_point.y);
                });
            }

            if (is_gui_mode)
            {
                cv::Mat dbgBackprojFrame;
                backproj_engine.getLastFrame(dbgBackprojFrame);
                cv::imshow("Backprojected frame", dbgBackprojFrame);

                for (auto target : allTargetBoundaries)
                {
                    //cv::circle(newColorFrame, cv::Point(target->lastTrackedPose.center.x, target->lastTrackedPose.center.y), 5, cv::Scalar(255), 3);
                    // TODO: Color if eliminated earlier
                    if (target->lastTrackedPose.size.area() > 0)
                        cv::ellipse(newColorFrame, target->lastTrackedPose, (hasTarget && target == std::get<0>(closest)) ? cv::Scalar(255, 0, 255) : cv::Scalar(255,255,0) , 1);
                }
            }

            xbot_vision::CubeTargetInfo targetInfo;

            if (!hasTarget) {
                targetInfo.has_tracked_cube = false;

                targetInfo.target_cube_point.header.stamp = ros::Time();
                targetInfo.target_cube_point.header.frame_id = "/gripper_middle";
                targetInfo.target_cube_point.point.x = 0;
                targetInfo.target_cube_point.point.y = 0;
                targetInfo.target_cube_point.point.z = 0;
            }
            else {
                //double headingRadians = trackedTarget->xOffset * (this->params.cameraHorizontalFovDegrees * PI / 180);
                //sl::float4 cube_position = get_position_for_target_rect(*trackedTarget->targetBounds.get(), newPointCloud);

                targetInfo.has_tracked_cube = true;
                //targetInfo.cube.distance_meters = distance;
                //targetInfo.cube.heading_deflection_radians = headingRadians;
                targetInfo.target_cube_point = std::get<1>(closest);
            }
            trackedCubePublisher.publish(targetInfo);
            visualizableTrackedCubePublisher.publish(targetInfo.target_cube_point);
        }

        if (is_gui_mode) {
            cv::imshow("Source", newColorFrame);
        }
    }
};

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

bool choose_file_name(std::string& out)
{
    struct stat st;
    if(stat("/data",&st) != 0) {
        ROS_INFO("Directory /data doesn't exist or isn't accessible");
        return false;
    }

    for (int file_id = 0; file_id < 20; file_id++)
    {
        out = "/data/zed_" + std::to_string(file_id) + ".svo";
        std::ifstream f(out.c_str());
        if (!f.good()) {
            return true;
        }
    }

    ROS_INFO("No available files after 20 tries");
    return false;
}

void rerun_training()
{
    std::string config_file_path;
    ros::param::get("/xbot_vision_node/config_file", config_file_path);

    VisionParams params;
    VisionParamsManager::loadParams(config_file_path, params);

    ColorBackProjectionEngine backproj_engine;
    backproj_engine.clear();

    XmlRpc::XmlRpcValue training_images_config;
    ros::param::get("/xbot_vision_node/training_images", training_images_config);

    /*for (int i = 0; i < training_images_config.size(); i++)
    {
        auto current_image = training_images_config[i];

        std::string image_path_rel = current_image["path"];
        std::string image_path = ros::package::getPath("xbot_vision") + "/../xbot_data/cube_training/" + image_path_rel;
        std::cout << "Opening " << image_path << std::endl;
        cv::Mat image = cv::imread(image_path);

        cv::Mat clahe_image;
        do_CLAHE_output_LAB(image, clahe_image);

        int x1 = static_cast<int>(current_image["a"][0]);
        int y1 = static_cast<int>(current_image["a"][1]);

        int x2 = static_cast<int>(current_image["b"][0]);
        int y2 = static_cast<int>(current_image["b"][1]);
        cv::Rect roi(x1, y1, (x2 - x1), (y2-y1));
        cv::Mat image_roi(clahe_image, roi);
        backproj_engine.updateHistFromTarget(image_roi);

        cv::rectangle(image, roi, cv::Scalar(255, 0, 0));
        imshow(std::to_string(i), image);

    }*/
    //backproj_engine.generateHistFromRange(cv::Rect2d(180, 85, 75, 50));
    backproj_engine.generateHistFromRange(cv::Rect2d(170, 95, 55, 27));
    backproj_engine.getTargetHist(params.targetHistogram);

    VisionParamsManager::saveParams(config_file_path, params);
    std::cout << "Saved params to " << config_file_path << std::endl;

    backproj_engine.displayHistogram();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xbot_vision_node");

    bool is_training = false;
    ros::param::get("/xbot_vision_node/is_training", is_training);
    if (is_training)
    {
        rerun_training();
        cv::waitKey(0);
        return 0;
    }

    ros::NodeHandle node;
    bool is_match_running = false;
    VisionNode vision_node(node, &is_match_running);

    /*cv::VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(CV_CAP_PROP_BUFFERSIZE, 3);

    if (!capture.isOpened())
        throw std::runtime_error("Capture not successfully opened!");*/

    // Create a ZED Camera object
    
    bool is_recording_initialized = false;
    sl::Camera zed;
    
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_HD720;
    init_params.depth_mode = sl::DEPTH_MODE_MEDIUM;
    init_params.coordinate_units = sl::UNIT_METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP; // ROS's coordinate system
    init_params.camera_fps = 30;

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS) {
        std::cout << sl::toString(err) << std::endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    zed.setCameraSettings(sl::CAMERA_SETTINGS_BRIGHTNESS, 4, false);
    zed.setCameraSettings(sl::CAMERA_SETTINGS_CONTRAST, 8, false);
    zed.setCameraSettings(sl::CAMERA_SETTINGS_HUE, 0, true);
    zed.setCameraSettings(sl::CAMERA_SETTINGS_SATURATION, 8, false);
    zed.setCameraSettings(sl::CAMERA_SETTINGS_GAIN, 30, true);
    zed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, -1, true);
    zed.setCameraSettings(sl::CAMERA_SETTINGS_AUTO_WHITEBALANCE, 0, false);
    zed.setCameraSettings(sl::CAMERA_SETTINGS_WHITEBALANCE, 2990, true);

    FpsCounter fpsCounter(10);

    vision_node.initialize();

    sl::RuntimeParameters runtime_params;
    runtime_params.sensing_mode = sl::SENSING_MODE_STANDARD;
    int width = zed.getResolution().width / 4;
    int height = zed.getResolution().height / 4;

    sl::Mat zedSourceFrame;
    sl::Mat zedPointCloud;
    for (uint32_t frame_number = 0; node.ok(); frame_number++)
    {
        if (is_match_running && !is_recording_initialized)
        {
            std::string file_name;
            if (choose_file_name(file_name)) {
                zed.enableRecording(file_name.c_str()); // By default, lossless compression
                ROS_INFO_STREAM("Initialized recording to file " << file_name);
                is_recording_initialized = true;
            }
            else {
                ROS_WARN("Failed to initialize recording");
            }
        }
        else if (!is_match_running && is_recording_initialized)
        {
            zed.disableRecording();
            is_recording_initialized = false;
            ROS_INFO("Stopped recording");
        }

        if (zed.grab(runtime_params) == sl::SUCCESS) {
            if (is_recording_initialized) {
                zed.record();
            }
            zed.retrieveImage(zedSourceFrame, sl::VIEW_LEFT, sl::MEM_CPU, width, height);
            zed.retrieveMeasure(zedPointCloud, sl::MEASURE_XYZRGBA, sl::MEM_CPU, width, height);
            cv::Mat sourceFrame = slMat2cvMat(zedSourceFrame);
            // capture.read(sourceFrame);
            vision_node.do_wait_key(sourceFrame);
            vision_node.process_frame_blob_and_shift_algorithm(frame_number, sourceFrame, zedPointCloud);
        }
        ros::spinOnce();

        fpsCounter.addFrameTimestamp(cv::getTickCount());
        if (frame_number % 30 * 5 == 0) {
            std::cout << "FPS: " << fpsCounter.getFps() << std::endl;
        }
    }

    if (is_recording_initialized)
    {
        zed.disableRecording();
    }

    zed.close();
    ros::shutdown();
}
