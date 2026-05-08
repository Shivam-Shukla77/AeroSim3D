#pragma once

#include <raylib.h>
#include <string>
#include <vector>
#include "physics.h"

extern Font fdoFont;

// UI Palette
const Color PANEL_BG = { 15, 15, 20, 240 };
const Color PANEL_BORDER = { 50, 50, 60, 255 };
const Color TERM_GREEN = { 57, 255, 20, 255 };
const Color TERM_AMBER = { 255, 176, 0, 255 };
const Color TERM_RED = { 255, 50, 50, 255 };
const Color TERM_DIM = { 140, 140, 150, 255 };

// Update the signature to accept Model rocketModel
void DrawDashboard(Rocket& rocket, const std::string& aiAnalysisText, Camera3D camera, Model rocketModel, Texture2D particleTex, Model launchPad, Vector3 padPosition, float padScale, Model terrainMap, Vector3& terrainPosition, float& terrainScale, const std::vector<Vector3>& flightPath, bool sasEnabled, RenderTexture2D minimapTex, RenderTexture2D viewCubeTex);

// Reusable UI components
bool DrawCommandButton(Rectangle bounds, const char* text);
void DrawTelemetryGraph(Rectangle bounds, const char* title, const std::vector<float>& data, float maxVal, Color lineColor);
void DrawScrollingGraph(Rectangle bounds, const std::vector<float>& data, Color lineColor, const char* label);
void DrawValueTweaker(Rectangle bounds, const char* label, float& value, float step, float minVal, float maxVal, const char* format);
void DrawHUDAttitudeIndicator(Rectangle bounds, float pitch, float roll, float yaw);
void DrawDebugAxes(Vector3 origin, Quaternion rotation, float length, float thickness);
