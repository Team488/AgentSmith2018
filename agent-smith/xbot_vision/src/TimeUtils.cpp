#include "TimeUtils.h"

#include <iostream>
#include <stdio.h>
#include <map>
#include <iomanip>
#include <opencv2/opencv.hpp>

TimerManager::TimerManager()
{
    addCheckpoint(TOTAL_CHECKPOINT_NAME);
}

TimerManager::~TimerManager()
{

}

void TimerManager::addCheckpoint(std::string name, std::string relativeTo)
{
    TimerCheckpoint newCheckpoint = { name, checkpoints.count(relativeTo) == 0 ? nullptr : &checkpoints.at(relativeTo), -1 };
    checkpoints[name] = newCheckpoint;
    orderedNames.push_back(name);
}

void TimerManager::printTableHeader()
{
    for (std::string name : orderedNames)
    {
        printf("%s ", name.c_str());
    }

    printf("\r\n");
}

void TimerManager::start()
{
    startTime = cv::getTickCount();
    for (std::string name : orderedNames)
    {
        checkpoints[name].endTime = -1;
    }
}

void TimerManager::markCheckpoint(std::string name)
{
    checkpoints[name].endTime = cv::getTickCount();
}

void TimerManager::stop()
{
    int64 endTime = cv::getTickCount();
    for (std::string name : orderedNames)
    {
        if (checkpoints[name].endTime < 0)
            checkpoints[name].endTime = endTime;
    }
}

void TimerManager::printTableRow(bool asOverwrite)
{
    if (asOverwrite)
        printf("\r");

    double totalTime = getCheckpointTime(checkpoints[TOTAL_CHECKPOINT_NAME]);

    for (std::string name : orderedNames)
    {
        //(std::to_string(checkpointTime) + std::to_string(std::round()) + "%").c_str()

        if (name == TOTAL_CHECKPOINT_NAME)
        {
            printf("%*d ", (int)name.size(), (int)totalTime);
        }
        else
        {
            double checkpointTime = getCheckpointTime(checkpoints[name]);

            std::ostringstream timeStr;
            timeStr << std::setprecision(3) << checkpointTime / totalTime * 100 << "%";

            printf("%*s ", (int)name.size(), timeStr.str().c_str());
        }
    }

    if (!asOverwrite)
        printf("\r\n");
}

double TimerManager::getCheckpointTime(TimerCheckpoint checkpoint)
{
    int64 localStartTime = checkpoint.relativeTo == nullptr ? startTime : checkpoint.relativeTo->endTime;
    return (checkpoint.endTime - localStartTime) / ((double)cvGetTickFrequency()*1000.);
}
