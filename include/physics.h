#pragma once

#include <raylib.h>
#include <raymath.h> // Required for Quaternion and Vector3 operations
#include <vector>

extern double global_M_earth;

struct Particle {
    Vector3 pos;
    Vector3 vel;
    float life;
    float maxLife;
    float size;
    float temperature;
    Vector3 thrustDir;
    bool active;
};

struct SmokeParticle {
    Vector3 pos;
    Vector3 vel;
    float life;
    float maxLife;
    float size;
    bool active;
};

struct PIDController {
    float kp, ki, kd;
    float integral;
    float prevError;
    
    float Update(float error, float dt);
    void Reset();
};

struct Rocket {
    // Vectors & Quaternions
    Vector3 position;
    Vector3 prevPosition;
    Vector3 lastWorldExitPos;
    Vector3 velocity;
    Vector3 angularVelocity;
    Quaternion orientation;
    PIDController pitchPID;
    PIDController yawPID;
    PIDController rollPID;
    Quaternion targetOrientation;
    bool autoPilotEnabled;
    bool autoLandEnabled; // Toggles the suicide burn auto-landing system
    // Mass & Tsiolkovsky
    float dryMass;
    float fuelMass;
    float maxFuelMass;
    float centerOfMassY;
    float specificImpulse;
    float massFlowRate;
    float availableDeltaV;
    // Aerodynamics
    float dragCoefficient;
    float crossSectionalArea;
    // Material Properties
    float restitution;
    float groundFriction;
    // Dynamics & UI
    float rotationalInertia;
    float momentOfInertia;
    float height;
    float rcsPower;
    float modelBottomOffset;
    float modelRadius;
    float throttle;
    Vector3 controlInput;
    Vector3 appliedTorque;
    float cameraDistance;
    float cameraHeight;
    // Thermodynamic & Fire System
    float combustionTemp;
    float fuelDensity;
    float exhaustVelocity;
    float plumeExpansion;
    float fireIntensity;
    static const int MAX_PARTICLES = 1024;
    Particle plume[MAX_PARTICLES];
    int activeParticleCount;
    float particleTimer;
    static const int MAX_SMOKE_PARTICLES = 2048;
    SmokeParticle smokePlume[MAX_SMOKE_PARTICLES];
    int activeSmokeCount;
    std::vector<float> altitudeHistory;
    std::vector<float> velocityHistory;
    Vector3 acceleration;
    float twr;
    float mach;
    float dynamicPressure;
};

// Updates the rocket's physics state based on forces and user input
// deltaTime: Time elapsed since the last frame
// totalTime: Total time elapsed since simulation start
void UpdatePhysics(Rocket& rocket, float deltaTime, float totalTime);

// Calculates the future trajectory path based on current velocity and spherical gravity
std::vector<Vector3> CalculateTrajectory(Rocket state, float timeAhead, int steps);