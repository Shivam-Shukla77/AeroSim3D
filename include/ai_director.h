#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include "physics.h"

class AIDirector {
public:
    AIDirector();
    ~AIDirector();

    void Start(Rocket* rocketPtr);
    void Stop();
    std::string GetLatestAnalysis();

private:
    std::thread workerThread;
    std::mutex dataMutex;
    std::atomic<bool> isRunning;
    std::string currentAnalysis;
    Rocket* telemetrySource;
};