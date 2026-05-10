#include <raylib.h>
#include <string>
#include <cmath> // For std::sqrt
#include <raymath.h>
#include <rlgl.h>
#include <vector>
#include <deque>
#include "physics.h"
#include "voxel_types.h"
#include "ui.h"
#include "ai_director.h"

Font fdoFont;

std::deque<float> altitudeHistory;
std::deque<float> velocityHistory;
float graphSampleTimer = 0.0f;
const int MAX_GRAPH_POINTS = 100;

AIDirector flightDirector;

int main() {
    // Setup: Initialize a Raylib window
    const int screenWidth = 1280;
    const int screenHeight = 720;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "AeroSim 3D");
    SetTargetFPS(60);

    RenderTexture2D minimapTarget = LoadRenderTexture(512, 512);
    if (minimapTarget.id == 0) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load resource [minimapTarget]");
        CloseWindow();
        return -1;
    }

    RenderTexture2D viewCubeTarget = LoadRenderTexture(256, 256);
    if (viewCubeTarget.id == 0) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load resource [viewCubeTarget]");
        UnloadRenderTexture(minimapTarget);
        CloseWindow();
        return -1;
    }

    // Set a massive far clipping plane
    rlSetClipPlanes(0.1, 100000.0);

    // Initialize Audio Device and Load Engine Sound
    InitAudioDevice();
    Music engineSound = LoadMusicStream("resources/freesound_community-065110_seamless-rocket-booster-roar-amp-crackle-42487.mp3");
    if (engineSound.stream.buffer == nullptr) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load resource [resources/freesound_community-065110_seamless-rocket-booster-roar-amp-crackle-42487.mp3]");
        CloseAudioDevice();
        UnloadRenderTexture(viewCubeTarget);
        UnloadRenderTexture(minimapTarget);
        CloseWindow();
        return -1;
    }
    engineSound.looping = true;
    PlayMusicStream(engineSound);

    // Initialization: Create an instance of the Rocket struct
    Rocket myRocket = { 0 };
    myRocket.position = { 0.0f, 0.0f, 0.0f };
    myRocket.prevPosition = { 0.0f, 0.0f, 0.0f };
    myRocket.lastWorldExitPos = { 0.0f, 0.0f, 0.0f };
    myRocket.velocity = { 0.0f, 0.0f, 0.0f };
    myRocket.dryMass = 25000.0f;
    myRocket.fuelMass = 100000.0f;
    myRocket.maxFuelMass = 100000.0f;
    myRocket.height = 20.0f;

    // Load dynamic voxel data for physics
    if (!GetOrInitializeVoxelGrid("resources/rocket_engine.glb", myRocket.voxelGrid, 10, myRocket.dryMass)) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load or precompute voxel data [resources/rocket_engine.glb]");
        UnloadMusicStream(engineSound);
        CloseAudioDevice();
        UnloadRenderTexture(viewCubeTarget);
        UnloadRenderTexture(minimapTarget);
        CloseWindow();
        return -1;
    }
    myRocket.dryMass = myRocket.voxelGrid.totalDryMass;

    myRocket.momentOfInertia = (1.0f / 12.0f) * myRocket.dryMass * (myRocket.height * myRocket.height);
    myRocket.specificImpulse = 320.0f; // e.g., 320 seconds of specific impulse
    myRocket.massFlowRate = 450.0f;    // 450 kg/s of fuel consumption
    myRocket.availableDeltaV = 0.0f;
    myRocket.dragCoefficient = 0.35f;
    myRocket.crossSectionalArea = 0.78f;
    myRocket.restitution = 0.15f;
    myRocket.groundFriction = 0.8f;
    myRocket.throttle = 0.0f;

    // Thermodynamic & Fire System
    myRocket.combustionTemp = 3200.0f; // Kelvin
    myRocket.fuelDensity = 810.0f; // RP-1 (kg/m^3)
    myRocket.exhaustVelocity = myRocket.specificImpulse * 9.81f; // Derived from Isp
    myRocket.plumeExpansion = 1.0f;
    myRocket.fireIntensity = 0.0f;

    myRocket.orientation = QuaternionIdentity(); // Initialize to no rotation
    myRocket.angularVelocity = {0.0f, 0.0f, 0.0f};
    myRocket.rotationalInertia = 5000.0f;
    myRocket.rcsPower = 250000.0f;
    myRocket.controlInput = { 0.0f, 0.0f, 0.0f };
    myRocket.cameraDistance = 40.0f;
    myRocket.cameraHeight = 10.0f;
    
    myRocket.pitchPID = { 2.5f, 0.1f, 1.5f, 0.0f, 0.0f };
    myRocket.yawPID = { 2.5f, 0.1f, 1.5f, 0.0f, 0.0f };
    myRocket.rollPID = { 2.5f, 0.1f, 1.5f, 0.0f, 0.0f };
    myRocket.autoPilotEnabled = false;
    myRocket.autoLandEnabled = false;

    // Initialize a 3D camera pointing at the rocket
    Camera3D camera = { 0 };
    camera.position = { 20.0f, 10.0f, 20.0f }; 
    camera.target = myRocket.position;     
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f; // Base cinematic FOV
    camera.projection = CAMERA_PERSPECTIVE;

    Camera3D mapCamera = { 0 };
    mapCamera.position = { myRocket.position.x, 50000.0f, myRocket.position.z }; // High in space
    mapCamera.target = myRocket.position;
    mapCamera.up = { 0.0f, 0.0f, -1.0f }; // Orient North
    mapCamera.fovy = 45.0f;
    mapCamera.projection = CAMERA_PERSPECTIVE;

    Camera3D cubeCamera = { 0 };
    cubeCamera.position = { 0.0f, 3.0f, -6.0f };
    cubeCamera.target = { 0.0f, 0.0f, 0.0f };
    cubeCamera.up = { 0.0f, 1.0f, 0.0f };
    cubeCamera.fovy = 35.0f;
    cubeCamera.projection = CAMERA_PERSPECTIVE;

    // Resource Loading
    Model rocketModel = LoadModel("resources/rocket_engine.glb");
    if (rocketModel.meshCount == 0 || rocketModel.meshes == nullptr) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load resource [resources/rocket_engine.glb]");
        UnloadMusicStream(engineSound);
        CloseAudioDevice();
        UnloadRenderTexture(viewCubeTarget);
        UnloadRenderTexture(minimapTarget);
        CloseWindow();
        return -1;
    }
    
    Model launchPad = LoadModel("resources/apollo_1_launch_site_memorial.glb");
    if (launchPad.meshCount == 0 || launchPad.meshes == nullptr) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load resource [resources/apollo_1_launch_site_memorial.glb]");
        UnloadModel(rocketModel);
        UnloadMusicStream(engineSound);
        CloseAudioDevice();
        UnloadRenderTexture(viewCubeTarget);
        UnloadRenderTexture(minimapTarget);
        CloseWindow();
        return -1;
    }
    Vector3 padPosition = { 0.0f, 0.0f, 0.0f };
    float padScale = 1.0f;

    Model terrainMap = LoadModel("resources/las_negras_terrain_model_almeria_spain (1).glb");
    if (terrainMap.meshCount == 0 || terrainMap.meshes == nullptr) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load resource [resources/las_negras_terrain_model_almeria_spain (1).glb]");
        UnloadModel(launchPad);
        UnloadModel(rocketModel);
        UnloadMusicStream(engineSound);
        CloseAudioDevice();
        UnloadRenderTexture(viewCubeTarget);
        UnloadRenderTexture(minimapTarget);
        CloseWindow();
        return -1;
    }
    Vector3 terrainPosition = { 300.0f, -30.0f, 0.0f };
    float terrainScale = 1.0f;

    // Load Global UI Font
    fdoFont = LoadFontEx("resources/Nasalization Rg.otf", 96, 0, 0);
    if (fdoFont.texture.id == 0) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load resource [resources/Nasalization Rg.otf]");
        UnloadModel(terrainMap); UnloadModel(launchPad); UnloadModel(rocketModel);
        UnloadMusicStream(engineSound); CloseAudioDevice();
        UnloadRenderTexture(viewCubeTarget); UnloadRenderTexture(minimapTarget);
        CloseWindow(); return -1;
    }
    SetTextureFilter(fdoFont.texture, TEXTURE_FILTER_BILINEAR);

    // Dynamic Initialization based on Model bounds
    BoundingBox bbox = GetModelBoundingBox(rocketModel);
    float modelWidthX = bbox.max.x - bbox.min.x;
    myRocket.modelRadius = modelWidthX / 2.0f;
    myRocket.modelBottomOffset = bbox.min.y;
    myRocket.crossSectionalArea = PI * myRocket.modelRadius * myRocket.modelRadius;

    // Generate Procedural Particle Texture
    Image particleImg = GenImageGradientRadial(64, 64, 0.0f, WHITE, BLANK);
    Texture2D particleTex = LoadTextureFromImage(particleImg);
    UnloadImage(particleImg); // Free RAM

    const float PHYSICS_STEP = 0.01f;
    float accumulator = 0.0f;
    bool flightMode = false;
    bool sasEnabled = true;

    static float camAzimuth = 0.0f;
    static float camElevation = 0.5f;
    static float camRadius = 50.0f;

    flightDirector.Start(&myRocket);

    // The Loop
    while (!WindowShouldClose()) {
        UpdateMusicStream(engineSound);

        if (IsKeyPressed(KEY_F11)) {
            // If we are resizing, it is safest to ensure the monitor is ready.
            int display = GetCurrentMonitor();
            if (IsWindowFullscreen()) {
                ToggleFullscreen();
                SetWindowSize(1280, 720); // Default windowed fallback
            } else {
                SetWindowSize(GetMonitorWidth(display), GetMonitorHeight(display));
                ToggleFullscreen();
            }
        }

        // Flight Mode / UI Mode Toggle
        if (IsKeyPressed(KEY_TAB)) {
            flightMode = !flightMode;
            if (flightMode) DisableCursor();
            else EnableCursor();
        }

        float distance = Vector3Distance(camera.position, myRocket.position);
        float distanceFactor = Clamp(50.0f / (distance + 1.0f), 0.0f, 1.0f);
        float audioAlt = fmax(0.0f, myRocket.position.y);
        float atmosphereFactor = Clamp(1.0f - (audioAlt / 100000.0f), 0.0f, 1.0f);
        
        // Target Calculations
        float targetVol = myRocket.throttle * distanceFactor * atmosphereFactor;
        float targetPitch = 0.5f + (0.7f * myRocket.throttle);

        // Spatial 3D Panning
        Vector3 toRocket = Vector3Normalize(Vector3Subtract(myRocket.position, camera.position));
        Vector3 camForward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
        Vector3 camRight = Vector3Normalize(Vector3CrossProduct(camForward, camera.up));
        float dotPan = Vector3DotProduct(toRocket, camRight);
        float targetPan = (dotPan + 1.0f) * 0.5f;

        // Apply Lerp Smoothing
        static float currentVol = 0.0f;
        static float currentPitch = 1.0f;
        static float currentPan = 0.5f;
        float lerpSpeed = GetFrameTime() * 5.0f;

        currentVol = Lerp(currentVol, targetVol, lerpSpeed);
        currentPitch = Lerp(currentPitch, targetPitch, lerpSpeed);
        currentPan = Lerp(currentPan, targetPan, lerpSpeed);

        // Apply to Hardware
        SetMusicVolume(engineSound, currentVol);
        SetMusicPitch(engineSound, currentPitch);
        SetMusicPan(engineSound, currentPan);

        float frameTime = GetFrameTime();
        if (frameTime > 0.25f) frameTime = 0.25f;
        accumulator += frameTime;

        if (flightMode) {
            if (IsKeyPressed(KEY_T)) sasEnabled = !sasEnabled;
            if (IsKeyPressed(KEY_H)) myRocket.autoLandEnabled = !myRocket.autoLandEnabled;

            // Camera Zoom
            camRadius -= GetMouseWheelMove() * 5.0f;
            if (camRadius < 10.0f) camRadius = 10.0f; // Prevent clipping through the model
        }


        while (accumulator >= PHYSICS_STEP) {
            UpdatePhysics(myRocket, PHYSICS_STEP, GetTime());
            accumulator -= PHYSICS_STEP;
        }

        // IJKL Auxiliary Camera Controls
        float camSpeed = 2.0f * GetFrameTime();
        if (IsKeyDown(KEY_J)) camAzimuth -= camSpeed;
        if (IsKeyDown(KEY_L)) camAzimuth += camSpeed;
        if (IsKeyDown(KEY_I)) camElevation += camSpeed;
        if (IsKeyDown(KEY_K)) camElevation -= camSpeed;
        camElevation = Clamp(camElevation, -1.5f, 1.5f); // Prevent flipping over the top

        // Calculate Position
        camera.target = myRocket.position;
        camera.position.x = myRocket.position.x + camRadius * cosf(camElevation) * sinf(camAzimuth);
        camera.position.y = myRocket.position.y + camRadius * sinf(camElevation);
        camera.position.z = myRocket.position.z + camRadius * cosf(camElevation) * cosf(camAzimuth);

        // Dynamic Field of View (FOV)
        float speed = Vector3Length(myRocket.velocity);
        float targetFOV = 60.0f + (speed * 0.1f);
        targetFOV = Clamp(targetFOV, 60.0f, 110.0f);
        camera.fovy = Lerp(camera.fovy, targetFOV, GetFrameTime() * 2.0f);

        // Camera Shake (Thrust Rattle)
        if (myRocket.throttle > 0.01f) {
            float shakeIntensity = myRocket.massFlowRate * myRocket.throttle * 0.002f; // Scale intensity by throttle and power
            camera.target.x += ((float)GetRandomValue(-100, 100) / 100.0f) * shakeIntensity;
            camera.target.y += ((float)GetRandomValue(-100, 100) / 100.0f) * shakeIntensity;
            camera.target.z += ((float)GetRandomValue(-100, 100) / 100.0f) * shakeIntensity;
        }

        // True Altitude Calculation
        double R_earth = 6371000.0;
        double earth_y = -R_earth;
        double rx = (double)myRocket.position.x;
        double ry = (double)myRocket.position.y - earth_y;
        double rz = (double)myRocket.position.z;
        float alt = (float)(std::sqrt(rx * rx + ry * ry + rz * rz) - R_earth);

        // Dynamic Color Lerp
        Color seaLevel = { 135, 206, 235, 255 }; // SkyBlue
        Color space = { 5, 5, 10, 255 };         // Deep Navy/Black
        float atmosphereRatio = Clamp(alt / 100000.0f, 0.0f, 1.0f);
        Color currentSky = {
            (unsigned char)Lerp((float)seaLevel.r, (float)space.r, atmosphereRatio),
            (unsigned char)Lerp((float)seaLevel.g, (float)space.g, atmosphereRatio),
            (unsigned char)Lerp((float)seaLevel.b, (float)space.b, atmosphereRatio),
            255
        };

        graphSampleTimer += GetFrameTime();
        if (graphSampleTimer >= 0.1f) {
            graphSampleTimer = 0.0f;
            altitudeHistory.push_back(myRocket.position.y);
            velocityHistory.push_back(Vector3Length(myRocket.velocity));
            // Keep the buffer from growing infinitely
            if (altitudeHistory.size() > MAX_GRAPH_POINTS) altitudeHistory.pop_front();
            if (velocityHistory.size() > MAX_GRAPH_POINTS) velocityHistory.pop_front();
        }

        std::vector<Vector3> flightPath = CalculateTrajectory(myRocket, 5.0f, 50);

        // Pass 1: The Minimap
        mapCamera.position.x = myRocket.position.x;
        mapCamera.position.y = myRocket.position.y + 25000.0f;
        mapCamera.position.z = myRocket.position.z;
        mapCamera.target = myRocket.position;
        
        BeginTextureMode(minimapTarget);
        ClearBackground(BLANK); // Distinct background
        BeginMode3D(mapCamera);
        if (flightPath.size() > 1) {
            for (size_t i = 0; i < flightPath.size() - 1; i++) {
                DrawLine3D(flightPath[i], flightPath[i+1], LIME);
            }
        }
        EndMode3D();

        // 2D Radar HUD Overlay
        for (int i = 0; i <= 512; i += 64) {
            DrawLine(i, 0, i, 512, Fade(SKYBLUE, 0.1f));
            DrawLine(0, i, 512, i, Fade(SKYBLUE, 0.1f));
        }
        
        DrawCircleLines(256, 256, 100, Fade(SKYBLUE, 0.4f));
        DrawCircleLines(256, 256, 200, Fade(SKYBLUE, 0.2f));
        DrawLine(256, 0, 256, 512, Fade(SKYBLUE, 0.3f));
        DrawLine(0, 256, 512, 256, Fade(SKYBLUE, 0.3f));

        // Radar Sweep Animation
        float sweepAngleDeg = (float)GetTime() * 120.0f; // Sweep at 120 degrees per second
        DrawCircleSector({256.0f, 256.0f}, 256.0f, sweepAngleDeg - 40.0f, sweepAngleDeg, 16, Fade(LIME, 0.15f));
        Vector2 sweepEnd = {
            256.0f + cosf(sweepAngleDeg * DEG2RAD) * 256.0f,
            256.0f + sinf(sweepAngleDeg * DEG2RAD) * 256.0f
        };
        DrawLineEx({256.0f, 256.0f}, sweepEnd, 2.0f, Fade(LIME, 0.8f));

        DrawTriangle({256.0f, 240.0f}, {246.0f, 266.0f}, {266.0f, 266.0f}, YELLOW);

        EndTextureMode();

        // Pass 2: The ViewCube
        BeginTextureMode(viewCubeTarget);
        ClearBackground(BLANK);
        BeginMode3D(cubeCamera);
        DrawCube({0.0f, 0.0f, 0.0f}, 2.0f, 2.0f, 2.0f, LIGHTGRAY);
        DrawCubeWires({0.0f, 0.0f, 0.0f}, 2.0f, 2.0f, 2.0f, DARKGRAY);
        DrawDebugAxes({0.0f, 0.0f, 0.0f}, myRocket.orientation, 2.5f, 0.15f);
        EndMode3D();
        EndTextureMode();

        BeginDrawing();
        ClearBackground(currentSky);

        // Render the UI and 3D scene
        DrawDashboard(myRocket, flightDirector.GetLatestAnalysis(), camera, rocketModel, particleTex, launchPad, padPosition, padScale, terrainMap, terrainPosition, terrainScale, flightPath, sasEnabled, minimapTarget, viewCubeTarget);

        EndDrawing();
    }

    // Resource Unloading
    UnloadFont(fdoFont);
    UnloadTexture(particleTex);
    UnloadModel(rocketModel);
    UnloadModel(launchPad);
    UnloadModel(terrainMap);

    UnloadRenderTexture(minimapTarget);
    UnloadRenderTexture(viewCubeTarget);

    UnloadMusicStream(engineSound);
    CloseAudioDevice();

    flightDirector.Stop();

    CloseWindow();
    return 0;
}