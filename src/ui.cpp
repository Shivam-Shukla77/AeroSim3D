#include "ui.h"
#include <cmath>
#include <rlgl.h>

extern std::vector<float> altitudeHistory;
extern std::vector<float> velocityHistory;

bool DrawCommandButton(Rectangle bounds, const char* text) {
    Vector2 mousePoint = GetMousePosition();
    bool isHovered = CheckCollisionPointRec(mousePoint, bounds);

    Color color = isHovered ? TERM_DIM : PANEL_BORDER;
    DrawRectangleRounded(bounds, 0.3f, 10, color);

    Vector2 textSize = MeasureTextEx(fdoFont, text, 20, 1);
    int textX = (int)(bounds.x + (bounds.width / 2.0f) - (textSize.x / 2.0f));
    int textY = (int)(bounds.y + (bounds.height / 2.0f) - (textSize.y / 2.0f));

    DrawTextEx(fdoFont, text, {(float)textX, (float)textY}, 20, 1, TERM_GREEN);

    return isHovered && IsMouseButtonPressed(MOUSE_LEFT_BUTTON);
}

void DrawTelemetryGraph(Rectangle bounds, const char* title, const std::vector<float>& data, float maxVal, Color lineColor) {
    Color panelBg = Fade(BLACK, 0.7f);
    Color panelBorder = Fade(SKYBLUE, 0.5f);
    Color textAccent = LIME;
    DrawRectangleRounded(bounds, 0.1f, 10, panelBg);
    DrawRectangleRoundedLines(bounds, 0.1f, 10, panelBorder);
    DrawTextEx(fdoFont, title, { bounds.x + 10.0f, bounds.y + 10.0f }, 20, 1, textAccent);

    if (data.empty()) return;

    Vector2 prevPoint = { 0.0f, 0.0f };
    for (size_t i = 0; i < data.size(); i++) {
        float x = bounds.x;
        if (data.size() > 1) {
            x += ((float)i / (data.size() - 1)) * bounds.width;
        }
        float normalizedY = Clamp(data[i] / maxVal, 0.0f, 1.0f);
        float y = (bounds.y + bounds.height) - (normalizedY * bounds.height);

        Vector2 currentPoint = { x, y };
        if (i > 0) {
            DrawLineEx(prevPoint, currentPoint, 2.0f, lineColor);
        }
        prevPoint = currentPoint;
    }
}

void DrawScrollingGraph(Rectangle bounds, const std::vector<float>& data, Color lineColor, const char* label) {
    Color panelBg = Fade(BLACK, 0.7f);
    Color panelBorder = Fade(SKYBLUE, 0.5f);
    
    DrawRectangleRounded(bounds, 0.1f, 10, panelBg);
    DrawRectangleRoundedLines(bounds, 0.1f, 10, panelBorder);
    
    float maxValue = 1.0f; // Prevent divide by zero
    for (float v : data) {
        if (v > maxValue) maxValue = v;
    }
    
    DrawTextEx(fdoFont, TextFormat("%s: %.1f", label, maxValue), { bounds.x + 10.0f, bounds.y + 10.0f }, 15, 1, LIME);
    
    if (data.size() < 2) return;
    
    // Matches the MAX_GRAPH_POINTS constant in main.cpp
    const int MAX_GRAPH_POINTS = 100;
    float xStep = bounds.width / MAX_GRAPH_POINTS;
    
    for (size_t i = 0; i < data.size() - 1; i++) {
        float y1 = bounds.y + bounds.height - ((data[i] / maxValue) * bounds.height);
        float y2 = bounds.y + bounds.height - ((data[i + 1] / maxValue) * bounds.height);
        
        DrawLineEx({bounds.x + (i * xStep), y1}, {bounds.x + ((i + 1) * xStep), y2}, 2.0f, lineColor);
    }
}

void DrawValueTweaker(Rectangle bounds, const char* label, float& value, float step, float minVal, float maxVal, const char* format) {
    // Background
    DrawRectangleRounded(bounds, 0.2f, 10, Fade(WHITE, 0.05f));

    // Text
    const char* valText = TextFormat(format, value);
    const char* fullText = TextFormat("%s: %s", label, valText);
    DrawTextEx(fdoFont, fullText, { bounds.x + 10.0f, bounds.y + (bounds.height / 2.0f) - 10.0f }, 20, 1, TERM_GREEN);

    // Buttons
    float btnSize = bounds.height - 4.0f; // 2px padding on top and bottom
    Rectangle minusBtn = { bounds.x + bounds.width - (btnSize * 2.0f) - 8.0f, bounds.y + 2.0f, btnSize, btnSize };
    Rectangle plusBtn = { bounds.x + bounds.width - btnSize - 4.0f, bounds.y + 2.0f, btnSize, btnSize };

    if (DrawCommandButton(minusBtn, "-")) value -= step;
    if (DrawCommandButton(plusBtn, "+")) value += step;

    value = Clamp(value, minVal, maxVal);
}

void DrawHUDAttitudeIndicator(Rectangle bounds, float pitch, float roll, float yaw) {
    DrawRectangleRec(bounds, Fade(BLACK, 0.6f));
    DrawRectangleLinesEx(bounds, 2.0f, Fade(SKYBLUE, 0.5f));

    BeginScissorMode((int)bounds.x, (int)bounds.y, (int)bounds.width, (int)bounds.height);

    Vector2 center = { bounds.x + bounds.width / 2.0f, bounds.y + bounds.height / 2.0f };
    rlPushMatrix();
    rlTranslatef(center.x, center.y, 0.0f);
    rlRotatef(roll, 0.0f, 0.0f, 1.0f);
    rlTranslatef(-center.x, -center.y, 0.0f);

    float pixelsPerDegree = 4.0f;
    float pitchOffset = pitch * pixelsPerDegree;

    for (int i = -90; i <= 90; i += 10) {
        float lineY = center.y + pitchOffset - (i * pixelsPerDegree);
        if (i == 0) {
            DrawLineEx({center.x - 40.0f, lineY}, {center.x + 40.0f, lineY}, 3.0f, WHITE);
        } else {
            DrawLineEx({center.x - 20.0f, lineY}, {center.x + 20.0f, lineY}, 2.0f, LIME);
        }
    }

    rlPopMatrix();

    // Compass Heading Ribbon
    DrawRectangleRec({bounds.x, bounds.y, bounds.width, 25.0f}, Fade(BLACK, 0.8f));
    DrawLineEx({bounds.x, bounds.y + 25.0f}, {bounds.x + bounds.width, bounds.y + 25.0f}, 2.0f, Fade(SKYBLUE, 0.5f));

    float yawPixelsPerDegree = bounds.width / 90.0f; // 90 degrees visible at once
    int startYaw = (int)std::floor(yaw - 45.0f);
    int endYaw = (int)std::ceil(yaw + 45.0f);

    for (int i = startYaw; i <= endYaw; i++) {
        if (i % 15 == 0) {
            float tickX = center.x + (i - yaw) * yawPixelsPerDegree;
            int displayHeading = i % 360;
            if (displayHeading < 0) displayHeading += 360;

            if (i % 45 == 0) {
                DrawLineEx({tickX, bounds.y}, {tickX, bounds.y + 8.0f}, 2.0f, WHITE);
                const char* dirText = (displayHeading == 0) ? "N" : (displayHeading == 90) ? "E" : (displayHeading == 180) ? "S" : (displayHeading == 270) ? "W" : TextFormat("%03d", displayHeading);
                
                Vector2 tSize = MeasureTextEx(fdoFont, dirText, 10, 1);
                DrawTextEx(fdoFont, dirText, { tickX - (tSize.x / 2.0f), bounds.y + 12.0f }, 10, 1, LIME);
            } else {
                DrawLineEx({tickX, bounds.y}, {tickX, bounds.y + 5.0f}, 1.0f, LIGHTGRAY);
            }
        }
    }

    // Compass Center Marker
    DrawTriangle({center.x - 5.0f, bounds.y + 25.0f}, {center.x + 5.0f, bounds.y + 25.0f}, {center.x, bounds.y + 20.0f}, YELLOW);

    // Static Boresight (Center Crosshair)
    DrawLineEx({center.x - 15.0f, center.y}, {center.x - 5.0f, center.y}, 3.0f, YELLOW);
    DrawLineEx({center.x + 5.0f, center.y}, {center.x + 15.0f, center.y}, 3.0f, YELLOW);
    DrawLineEx({center.x, center.y - 5.0f}, {center.x, center.y + 5.0f}, 3.0f, YELLOW);

    EndScissorMode();
}

void DrawDebugAxes(Vector3 origin, Quaternion rotation, float length, float thickness) {
    Vector3 right = Vector3RotateByQuaternion({1.0f, 0.0f, 0.0f}, rotation);
    Vector3 up = Vector3RotateByQuaternion({0.0f, 1.0f, 0.0f}, rotation);
    Vector3 forward = Vector3RotateByQuaternion({0.0f, 0.0f, 1.0f}, rotation);

    DrawCylinderEx(origin, Vector3Add(origin, Vector3Scale(right, length)), thickness, thickness, 8, RED);
    DrawCylinderEx(origin, Vector3Add(origin, Vector3Scale(up, length)), thickness, thickness, 8, GREEN);
    DrawCylinderEx(origin, Vector3Add(origin, Vector3Scale(forward, length)), thickness, thickness, 8, BLUE);
}

void DrawDashboard(Rocket& rocket, const std::string& aiAnalysisText, Camera3D camera, Model rocketModel, Texture2D particleTex, Model launchPad, Vector3 padPosition, float padScale, Model terrainMap, Vector3& terrainPosition, float& terrainScale, const std::vector<Vector3>& flightPath, bool sasEnabled, RenderTexture2D minimapTex, RenderTexture2D viewCubeTex) {
    BeginMode3D(camera);

    // Render Terrain
    DrawModel(terrainMap, terrainPosition, terrainScale, WHITE);

    // Render Launch Pad
    DrawModel(launchPad, padPosition, padScale, WHITE);

    // Render Trajectory Path
    if (flightPath.size() > 1) {
        for (size_t i = 0; i < flightPath.size() - 1; i++) {
            DrawLine3D(flightPath[i], flightPath[i+1], Fade(LIME, 0.5f));
        }
    }

    // Calculate Axis and Angle for 3D Rotation
    Vector3 rotAxis;
    float rotAngle;
    QuaternionToAxisAngle(rocket.orientation, &rotAxis, &rotAngle);
    float rotAngleDeg = rotAngle * RAD2DEG; // DrawModelEx requires degrees

    // Calculate heat intensity for reentry plasma
    float heatIntensity = 0.0f;
    if (rocket.mach > 2.5f && rocket.dynamicPressure > 500.0f) {
        float mFactor = Clamp((rocket.mach - 2.5f) / 3.0f, 0.0f, 1.0f); // Max heat at Mach 5.5+
        float pFactor = Clamp(rocket.dynamicPressure / 20000.0f, 0.0f, 1.0f); // Max heat at 20kPa+
        heatIntensity = mFactor * pFactor;
    }

    // Tint the rocket model based on heating
    Color modelTint = WHITE;
    if (heatIntensity > 0.0f) {
        modelTint.g = (unsigned char)Lerp(255.0f, 100.0f, heatIntensity);
        modelTint.b = (unsigned char)Lerp(255.0f, 50.0f, heatIntensity);
    }

    // Render the rocket using DrawModelEx instead of DrawCylinder
    DrawModelEx(rocketModel, rocket.position, rotAxis, rotAngleDeg, {1.0f, 1.0f, 1.0f}, modelTint);

    // Render Reentry Plasma Glow
    if (heatIntensity > 0.01f) {
        rlDisableBackfaceCulling();
        rlDisableDepthMask();
        BeginBlendMode(BLEND_ADDITIVE);

        // Position the plasma shield on the leading edge based on velocity direction
        Vector3 velDir = Vector3Normalize(rocket.velocity);
        Vector3 plasmaPos = Vector3Add(rocket.position, Vector3Scale(velDir, rocket.height * 0.4f));

        Color plasmaOuter = { (unsigned char)255, (unsigned char)100, (unsigned char)0, (unsigned char)(150.0f * heatIntensity) };
        Color plasmaInner = { (unsigned char)255, (unsigned char)200, (unsigned char)100, (unsigned char)(255.0f * heatIntensity) };

        float shieldRadius = rocket.modelRadius * Lerp(1.2f, 1.8f, heatIntensity);
        DrawSphere(plasmaPos, shieldRadius, plasmaOuter);
        DrawSphere(plasmaPos, shieldRadius * 0.7f, plasmaInner);

        float flicker = (float)GetRandomValue(90, 110) / 100.0f;
        DrawSphere(plasmaPos, shieldRadius * 0.4f * flicker, WHITE);

        EndBlendMode();
        rlEnableDepthMask();
        rlEnableBackfaceCulling();
    }

    // Visual Aerodynamic Vapor Trail (Mach Cone)
    if (rocket.mach > 0.8f && rocket.mach < 1.2f && rocket.dynamicPressure > 5000.0f) {
        // Calculate an intensity curve that peaks exactly at Mach 1.0
        float machFactor = 1.0f - std::abs(rocket.mach - 1.0f) * 5.0f;
        machFactor = Clamp(machFactor, 0.0f, 1.0f);
        
        // Ensure it only appears where the air is thick enough
        float pressureFactor = Clamp(rocket.dynamicPressure / 20000.0f, 0.0f, 1.0f);
        float intensity = machFactor * pressureFactor;

        if (intensity > 0.01f) {
            rlDisableBackfaceCulling();
            rlDisableDepthMask();
            BeginBlendMode(BLEND_ADDITIVE);

            Vector3 coneTopLocal = { 0.0f, rocket.height * 0.3f, 0.0f }; // Slightly below the nose
            Vector3 coneBottomLocal = { 0.0f, -rocket.height * 0.2f, 0.0f }; // Mid-body
            Vector3 coneTop = Vector3Add(rocket.position, Vector3RotateByQuaternion(coneTopLocal, rocket.orientation));
            Vector3 coneBottom = Vector3Add(rocket.position, Vector3RotateByQuaternion(coneBottomLocal, rocket.orientation));

            float topRadius = rocket.modelRadius * 1.05f; // Hug the hull
            float bottomRadius = rocket.modelRadius * 3.5f; // Flare outwards
            Color vaporColor = { (unsigned char)200, (unsigned char)230, (unsigned char)255, (unsigned char)(100.0f * intensity) };

            DrawCylinderEx(coneTop, coneBottom, topRadius, bottomRadius, 32, vaporColor);

            EndBlendMode();
            rlEnableDepthMask();
            rlEnableBackfaceCulling();
        }
    }

    // Render Engine Exhaust (CPU Particle System)
    rlDisableBackfaceCulling();
    rlDisableDepthMask();

    // Render Smoke Plume (Alpha Blending)
    BeginBlendMode(BLEND_ALPHA);
    for (int i = 0; i < Rocket::MAX_SMOKE_PARTICLES; i++) {
        if (rocket.smokePlume[i].active && rocket.smokePlume[i].life > 0.0f) {
            float lifeRatio = rocket.smokePlume[i].life / rocket.smokePlume[i].maxLife;
            lifeRatio = Clamp(lifeRatio, 0.0f, 1.0f);
            
            Color sColor = { (unsigned char)100, (unsigned char)100, (unsigned char)110, (unsigned char)255 }; // Thick industrial gray smoke
            float alpha = (lifeRatio > 0.9f) ? (1.0f - lifeRatio) * 10.0f : lifeRatio; // Fade in quickly, fade out slowly
            sColor.a = (unsigned char)(150.0f * alpha);
            
            DrawBillboard(camera, particleTex, rocket.smokePlume[i].pos, rocket.smokePlume[i].size, sColor);
        }
    }
    EndBlendMode();

    BeginBlendMode(BLEND_ADDITIVE);

    Vector3 localOffset = {0.0f, rocket.modelBottomOffset, 0.0f};
    Vector3 rotatedOffset = Vector3RotateByQuaternion(localOffset, rocket.orientation);
    Vector3 nozzlePos = Vector3Add(rocket.position, rotatedOffset);

    for (int i = 0; i < Rocket::MAX_PARTICLES; i++) {
        if (rocket.plume[i].active && rocket.plume[i].life > 0.0f) {
            float lifeRatio = rocket.plume[i].life / rocket.plume[i].maxLife;
            lifeRatio = Clamp(lifeRatio, 0.0f, 1.0f);
            
            float size = rocket.modelRadius * Lerp(5.0f, 1.2f, lifeRatio);
            
            Color pColor = WHITE;
            if (lifeRatio < 0.7f) pColor = ORANGE;
            if (lifeRatio < 0.3f) pColor = DARKGRAY;
            pColor.a = (unsigned char)(255.0f * (lifeRatio * lifeRatio));
            
            DrawBillboard(camera, particleTex, rocket.plume[i].pos, size, pColor);
        }
    }

    EndBlendMode();
    rlEnableDepthMask();
    rlEnableBackfaceCulling();

    EndMode3D();

    Color panelBg = Fade(BLACK, 0.7f);
    Color panelBorder = Fade(SKYBLUE, 0.5f);
    Color textAccent = LIME;

    // The Telemetry Panel (Left Side)
    Rectangle telemetryRec = { 20, 20, 400, 450 };
    DrawRectangleRounded(telemetryRec, 0.1f, 10, panelBg);
    DrawRectangleRoundedLines(telemetryRec, 0.1f, 10, panelBorder);
    DrawTextEx(fdoFont, "FLIGHT DYNAMICS OVERVIEW", { 30.0f, 27.0f }, 10, 1, WHITE);

    double R_earth = 6371000.0;
    double earth_y = -R_earth;
    double rx = (double)rocket.position.x;
    double ry = (double)rocket.position.y - earth_y;
    double rz = (double)rocket.position.z;
    float altitude = (float)(std::sqrt(rx * rx + ry * ry + rz * rz) - R_earth);

    // KINEMATICS
    DrawTextEx(fdoFont, "--- KINEMATICS ---", { 30.0f, 70.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, "ALTITUDE:", { 30.0f, 90.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f m", altitude), { 150.0f, 90.0f }, 10, 1, textAccent);
    DrawTextEx(fdoFont, "VELOCITY:", { 30.0f, 110.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f m/s", Vector3Length(rocket.velocity)), { 150.0f, 110.0f }, 10, 1, textAccent);
    DrawTextEx(fdoFont, "MACH NUMBER:", { 30.0f, 130.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f", rocket.mach), { 150.0f, 130.0f }, 10, 1, textAccent);
    DrawTextEx(fdoFont, "ACCELERATION:", { 30.0f, 150.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f G", Vector3Length(rocket.acceleration) / 9.8f), { 150.0f, 150.0f }, 10, 1, textAccent);

    // DYNAMICS
    DrawTextEx(fdoFont, "--- DYNAMICS ---", { 30.0f, 170.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, "TOTAL MASS:", { 30.0f, 190.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f kg", rocket.dryMass + rocket.fuelMass), { 150.0f, 190.0f }, 10, 1, textAccent);
    DrawTextEx(fdoFont, "FUEL MASS:", { 30.0f, 210.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f kg", rocket.fuelMass), { 150.0f, 210.0f }, 10, 1, textAccent);
    DrawTextEx(fdoFont, "CENTER OF MASS:", { 30.0f, 230.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("Y: %6.2f m", rocket.centerOfMassY), { 150.0f, 230.0f }, 10, 1, textAccent);
    DrawTextEx(fdoFont, "TWR:", { 30.0f, 250.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f", rocket.twr), { 150.0f, 250.0f }, 10, 1, rocket.twr < 1.0f ? TERM_RED : textAccent);

    // FORCES & ATTITUDE
    DrawTextEx(fdoFont, "--- FORCES & ATTITUDE ---", { 30.0f, 290.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, "DYNAMIC PRESS:", { 30.0f, 310.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("%8.2f Pa", rocket.dynamicPressure), { 150.0f, 310.0f }, 10, 1, textAccent);
    
    Vector3 euler = QuaternionToEuler(rocket.orientation);
    DrawTextEx(fdoFont, "ATTITUDE:", { 30.0f, 330.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, TextFormat("PITCH: %6.1f  YAW: %6.1f", euler.x * RAD2DEG, euler.y * RAD2DEG), { 150.0f, 330.0f }, 10, 1, textAccent);
    DrawTextEx(fdoFont, "AUTO-LAND:", { 30.0f, 350.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, rocket.autoLandEnabled ? "ARMED" : "OFF", { 150.0f, 350.0f }, 10, 1, rocket.autoLandEnabled ? TERM_GREEN : TERM_DIM);

    // Primary Flight Display (PFD) HUD
    Rectangle hudBounds = { telemetryRec.x + 260.0f, telemetryRec.y + 240.0f, 120.0f, 120.0f };
    DrawHUDAttitudeIndicator(hudBounds, euler.x * RAD2DEG, euler.z * RAD2DEG, euler.y * RAD2DEG);

    DrawTextEx(fdoFont, "AI ANALYSIS:", { 30.0f, 370.0f }, 10, 1, TERM_DIM);
    DrawTextEx(fdoFont, aiAnalysisText.c_str(), { 30.0f, 390.0f }, 10, 1, textAccent);

    // Orbital Minimap
    int screenW = GetScreenWidth();
    int screenH = GetScreenHeight();
    Rectangle mapRec = { (float)(screenW - 220), (float)(screenH - 220), 200.0f, 200.0f };
    DrawRectangleRounded(mapRec, 0.1f, 10, Fade(BLACK, 0.8f));
    Rectangle sourceRec = { 0.0f, 0.0f, (float)minimapTex.texture.width, -(float)minimapTex.texture.height };
    DrawTexturePro(minimapTex.texture, sourceRec, mapRec, {0.0f, 0.0f}, 0.0f, WHITE);
    DrawRectangleRoundedLines(mapRec, 0.1f, 10, panelBorder);

    // Compass Overlay for Minimap
    Vector2 mapCenter = { mapRec.x + mapRec.width / 2.0f, mapRec.y + mapRec.height / 2.0f };
    DrawTextEx(fdoFont, "N", { mapCenter.x - MeasureTextEx(fdoFont, "N", 15, 1).x / 2.0f, mapRec.y - 20.0f }, 15, 1, WHITE);
    DrawTextEx(fdoFont, "S", { mapCenter.x - MeasureTextEx(fdoFont, "S", 15, 1).x / 2.0f, mapRec.y + mapRec.height + 5.0f }, 15, 1, WHITE);
    DrawTextEx(fdoFont, "E", { mapRec.x + mapRec.width + 10.0f, mapCenter.y - MeasureTextEx(fdoFont, "E", 15, 1).y / 2.0f }, 15, 1, WHITE);
    DrawTextEx(fdoFont, "W", { mapRec.x - MeasureTextEx(fdoFont, "W", 15, 1).x - 10.0f, mapCenter.y - MeasureTextEx(fdoFont, "W", 15, 1).y / 2.0f }, 15, 1, WHITE);

    // Attitude ViewCube
    Rectangle cubeRec = { mapRec.x - 220.0f, mapRec.y + 50.0f, 200.0f, 150.0f };
    DrawRectangleRounded(cubeRec, 0.1f, 10, Fade(BLACK, 0.8f));
    Rectangle cubeSourceRec = { 0.0f, 0.0f, (float)viewCubeTex.texture.width, -(float)viewCubeTex.texture.height };
    DrawTexturePro(viewCubeTex.texture, cubeSourceRec, cubeRec, {0.0f, 0.0f}, 0.0f, WHITE);
    DrawRectangleRoundedLines(cubeRec, 0.1f, 10, panelBorder);
    DrawTextEx(fdoFont, "ATTITUDE CUBE", { cubeRec.x + 10.0f, cubeRec.y + 10.0f }, 10, 1, WHITE);

    // Telemetry Graphs
    float screenHeight = (float)GetScreenHeight();
    Rectangle altBounds = { 20.0f, screenHeight - 230.0f, 400.0f, 100.0f };
    Rectangle velBounds = { 20.0f, screenHeight - 120.0f, 400.0f, 100.0f };
    DrawScrollingGraph(altBounds, altitudeHistory, SKYBLUE, "ALTITUDE (m)");
    DrawScrollingGraph(velBounds, velocityHistory, textAccent, "VELOCITY (m/s)");

    // Sandbox Editor Panel
    float screenWidth = (float)GetScreenWidth();
    Rectangle editorBounds = { screenWidth - 370.0f, 20.0f, 350.0f, 500.0f };
    DrawRectangleRounded(editorBounds, 0.1f, 10, panelBg);
    DrawRectangleRoundedLines(editorBounds, 0.1f, 10, panelBorder);
    DrawTextEx(fdoFont, "SANDBOX EDITOR", { editorBounds.x + 20.0f, editorBounds.y + 15.0f }, 20, 1, textAccent);

    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 50, 330, 30}, "Dry Mass (kg)", rocket.dryMass, 1000.0f, 500.0f, 100000.0f, "%.0f");
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 90, 330, 30}, "Fuel (kg)", rocket.fuelMass, 5000.0f, 0.0f, 500000.0f, "%.0f");

    float altTweak = rocket.position.y;
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 130, 330, 30}, "Altitude (m)", altTweak, 1000.0f, 0.0f, 1000000.0f, "%.0f");
    if (altTweak != rocket.position.y) { 
        rocket.position.y = altTweak; 
        rocket.velocity = {0.0f, 0.0f, 0.0f}; 
        rocket.altitudeHistory.clear(); 
    }

    static float earthGravityMulti = 1.0f;
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 170, 330, 30}, "Earth Gravity (x)", earthGravityMulti, 0.1f, 0.0f, 10.0f, "%.1f");
    global_M_earth = 5.9722e24 * (double)earthGravityMulti;

    // Terrain Controls
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 210, 330, 30}, "Terrain Scale", terrainScale, 1.0f, 0.1f, 10000.0f, "%.1f");
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 250, 330, 30}, "Terrain X Offset", terrainPosition.x, 100.0f, -50000.0f, 50000.0f, "%.0f");
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 290, 330, 30}, "Terrain Y Offset", terrainPosition.y, 100.0f, -5000.0f, 5000.0f, "%.0f");

    // PID Controls
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 330, 330, 30}, "PID P (Proportional)", rocket.pitchPID.kp, 0.1f, 0.0f, 20.0f, "%.1f");
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 370, 330, 30}, "PID I (Integral)", rocket.pitchPID.ki, 0.01f, 0.0f, 5.0f, "%.2f");
    DrawValueTweaker({editorBounds.x + 10, editorBounds.y + 410, 330, 30}, "PID D (Derivative)", rocket.pitchPID.kd, 0.1f, 0.0f, 20.0f, "%.1f");

    // Sync yaw and roll PID to match pitch PID
    rocket.yawPID.kp = rocket.rollPID.kp = rocket.pitchPID.kp;
    rocket.yawPID.ki = rocket.rollPID.ki = rocket.pitchPID.ki;
    rocket.yawPID.kd = rocket.rollPID.kd = rocket.pitchPID.kd;

    // Resolution Cycler
    static const int resList[3][2] = { {1280, 720}, {1920, 1080}, {2560, 1440} };
    static int currentRes = 0;

    Rectangle resButton = {editorBounds.x + 10, editorBounds.y + 450, 330, 35};
    const char* resText = TextFormat("RESOLUTION: %dx%d", resList[currentRes][0], resList[currentRes][1]);
    if (DrawCommandButton(resButton, resText)) {
        currentRes = (currentRes + 1) % 3;
        if (IsWindowFullscreen()) ToggleFullscreen(); // Safest to exit FS before forcing a resize
        SetWindowSize(resList[currentRes][0], resList[currentRes][1]);
        SetWindowPosition(GetMonitorWidth(GetCurrentMonitor())/2 - resList[currentRes][0]/2, GetMonitorHeight(GetCurrentMonitor())/2 - resList[currentRes][1]/2); // Center window
    }
}
