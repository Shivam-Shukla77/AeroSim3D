#include "ai_director.h"
#include <iostream>
#include <chrono>
#include <vector>

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #define NOGDI
    #define NOUSER
#endif
#include <curl/curl.h>
#include <raymath.h> // For Vector3Length

// For JSON manipulation. Assumes nlohmann/json is in the include path.
// Get it from: https://github.com/nlohmann/json
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Libcurl write callback function to capture the HTTP response
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

static std::string WrapText(const std::string& text, size_t line_length) {
    std::string result = text;
    size_t last_space = 0;
    size_t line_start = 0;
    
    for (size_t i = 0; i < result.length(); ++i) {
        if (result[i] == ' ') last_space = i;
        
        if (i - line_start > line_length && last_space > line_start) {
            result[last_space] = '\n';
            line_start = last_space + 1;
        }
    }
    return result;
}

AIDirector::AIDirector() : isRunning(false), currentAnalysis("Awaiting telemetry..."), telemetrySource(nullptr) {}

AIDirector::~AIDirector() {
    Stop();
}

std::string AIDirector::GetLatestAnalysis() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return currentAnalysis;
}

void AIDirector::Stop() {
    isRunning = false;
    if (workerThread.joinable()) {
        workerThread.join();
    }
}

void AIDirector::Start(Rocket* rocketPtr) {
    if (isRunning) {
        return; // Already running
    }
    telemetrySource = rocketPtr;
    isRunning = true;

    // Launch the worker thread with a lambda function for the main loop
    workerThread = std::thread([this]() {
        CURL* curl = curl_easy_init();
        if (!curl) {
            std::cerr << "Error: Failed to initialize libcurl." << std::endl;
            return;
        }

        while (isRunning) {
            // a. Sleep to avoid spamming the AI service
            std::this_thread::sleep_for(std::chrono::seconds(3));

            if (!telemetrySource || !isRunning) {
                continue;
            }

            // b. Safely read telemetry data
            float velocity = Vector3Length(telemetrySource->velocity);
            double R_earth = 6371000.0;
            double earth_y = -R_earth;
            double rx = (double)telemetrySource->position.x;
            double ry = (double)telemetrySource->position.y - earth_y;
            double rz = (double)telemetrySource->position.z;
            float altitude = (float)(std::sqrt(rx * rx + ry * ry + rz * rz) - R_earth);

            // c. Construct the JSON payload with the specified prompt
            std::string prompt = "You are a NASA Flight Director. Telemetry: Alt " + std::to_string((int)altitude) +
                                 "m, Vel " + std::to_string((int)velocity) +
                                 "m/s. Give a realistic 2-sentence mission control status update. Use technical aerospace jargon. No markdown.";

            json payload = {
                {"model", "gemma4:e2b"},
                {"prompt", prompt},
                {"stream", false}
            };
            std::string payloadStr = payload.dump();

            // d. Use libcurl to send the POST request
            std::string readBuffer;
            struct curl_slist* headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:11434/api/generate");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payloadStr.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 240L); // Prevent long hangs

            CURLcode res = curl_easy_perform(curl);

            // e. Parse the response and update the analysis string
            if (res == CURLE_OK) {
                try {
                    json responseJson = json::parse(readBuffer);
                    if (responseJson.contains("response")) {
                        std::string analysis = responseJson["response"];
                        // Clean up potential newlines from the response
                        analysis.erase(std::remove(analysis.begin(), analysis.end(), '\n'), analysis.end());
                        analysis.erase(std::remove(analysis.begin(), analysis.end(), '*'), analysis.end());
                        
                        analysis = WrapText(analysis, 45);

                        std::lock_guard<std::mutex> lock(dataMutex);
                        currentAnalysis = analysis;
                    }
                } catch (json::parse_error& e) {
                    std::lock_guard<std::mutex> lock(dataMutex);
                    currentAnalysis = "AI: Response error.";
                }
            } else {
                std::lock_guard<std::mutex> lock(dataMutex);
                currentAnalysis = "AI: Connection failed.";
            }

            curl_slist_free_all(headers);
        }

        curl_easy_cleanup(curl);
    });
}