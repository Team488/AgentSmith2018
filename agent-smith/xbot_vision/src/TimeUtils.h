#pragma once

#include <stdint.h>
#include <string>
#include <map>
#include <vector>

#define TOTAL_CHECKPOINT_NAME "TOTAL"

struct TimerCheckpoint {
    std::string name;
    TimerCheckpoint* relativeTo;
    int64_t endTime;
};

class TimerManager
{
private:
    std::map<std::string, TimerCheckpoint> checkpoints;
    std::vector<std::string> orderedNames;
    int64_t startTime;

public:
    TimerManager();
    ~TimerManager();

    void addCheckpoint(std::string name, std::string relativeTo = "");
    
    void printTableHeader();

    void start();
    void markCheckpoint(std::string name);
    void stop();
    double getCheckpointTime(TimerCheckpoint checkpoint);
    void printTableRow(bool asOverwrite = true);
};
