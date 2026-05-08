#include "physics.h"
#include <cmath> // For std::sqrt, std::exp
#define FNL_IMPL
#include "FastNoiseLite.h"

double global_M_earth = 5.9722e24;

struct State { 
    Vector3 pos; 
    Vector3 vel; 
    float mass; 
};

struct Derivative { 
    Vector3 vel; 
    Vector3 accel; 
    float massRate; 
};

float PIDController::Update(float error, float dt) {
    if (dt <= 0.0f) return 0.0f;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
}

void PIDController::Reset() {
    integral = 0.0f;
    prevError = 0.0f;
}

// Evaluates the forces and returns the derivative for the RK4 step
Derivative Evaluate(const Rocket& baseParams, const State& initial, float t, float dt, const Derivative& d) {
    State state;
    state.pos.x = initial.pos.x + d.vel.x * dt;
    state.pos.y = initial.pos.y + d.vel.y * dt;
    state.pos.z = initial.pos.z + d.vel.z * dt;

    state.vel.x = initial.vel.x + d.accel.x * dt;
    state.vel.y = initial.vel.y + d.accel.y * dt;
    state.vel.z = initial.vel.z + d.accel.z * dt;

    state.mass = initial.mass + d.massRate * dt;

    Vector3 netForce = {0.0f, 0.0f, 0.0f};

    // Spherical Coordinate Setup & Gravity
    double G = 6.67430e-11;
    double R_earth = 6371000.0;
    double earth_y = -R_earth;

    double rx = (double)state.pos.x;
    double ry = (double)state.pos.y - earth_y;
    double rz = (double)state.pos.z;
    double r_mag = std::sqrt(rx * rx + ry * ry + rz * rz);
    
    double gravityMag = (G * global_M_earth * state.mass) / (r_mag * r_mag);
    netForce.x += (float)(-rx / r_mag * gravityMag);
    netForce.y += (float)(-ry / r_mag * gravityMag);
    netForce.z += (float)(-rz / r_mag * gravityMag);

    // Procedural Wind & Atmosphere
    static fnl_state noise = fnlCreateState();
    noise.noise_type = FNL_NOISE_OPENSIMPLEX2;

    double trueAltitude = r_mag - R_earth;
    
    // Wind shear profile: Stronger winds (jet stream) around 12km altitude
    float jetStreamFactor = std::exp(-std::pow(((float)trueAltitude - 12000.0f) / 4000.0f, 2.0f));
    float windSpeed = 10.0f + (65.0f * jetStreamFactor); // Up to 75 m/s crosswinds

    Vector3 wind;
    wind.x = fnlGetNoise2D(&noise, state.pos.y * 0.001f, t * 0.1f) * windSpeed; 
    wind.y = 0.0f; 
    wind.z = fnlGetNoise2D(&noise, state.pos.x * 0.001f, t * 0.1f) * windSpeed;

    // True Altitude & Barometric Drag
    if (trueAltitude < 100000.0) {
        float airDensity = 1.225f * std::exp((float)-trueAltitude / 8500.0f);
        Vector3 relVel = Vector3Subtract(state.vel, wind);

        // Decompose relative velocity into axial and lateral vectors
        Vector3 rocketUp = Vector3RotateByQuaternion({0.0f, 1.0f, 0.0f}, baseParams.orientation);
        float axialSpeed = Vector3DotProduct(relVel, rocketUp);
        Vector3 axialVel = Vector3Scale(rocketUp, axialSpeed);
        Vector3 lateralVel = Vector3Subtract(relVel, axialVel);

        // Axial Drag (Frontal cross-section)
        float axialSpeedSq = Vector3LengthSqr(axialVel);
        if (axialSpeedSq > 0.001f) {
            float axialDragMag = 0.5f * airDensity * axialSpeedSq * baseParams.dragCoefficient * baseParams.crossSectionalArea;
            Vector3 axialDragDir = Vector3Normalize(Vector3Negate(axialVel));
            netForce = Vector3Add(netForce, Vector3Scale(axialDragDir, axialDragMag));
        }

        // Aerodynamic Wind Shear (Lateral force on the broad side of the rocket)
        float lateralSpeedSq = Vector3LengthSqr(lateralVel);
        if (lateralSpeedSq > 0.001f) {
            float sideArea = baseParams.height * (baseParams.modelRadius * 2.0f);
            float lateralDragMag = 0.5f * airDensity * lateralSpeedSq * baseParams.dragCoefficient * sideArea;
            Vector3 lateralDragDir = Vector3Normalize(Vector3Negate(lateralVel));
            netForce = Vector3Add(netForce, Vector3Scale(lateralDragDir, lateralDragMag));
        }
    }

    // Calculate Thrust Force (Tsiolkovsky rocket equation)
    float mRate = 0.0f;
    if (baseParams.throttle > 0.01f && state.mass > baseParams.dryMass) {
        float actualMassFlow = baseParams.massFlowRate * baseParams.throttle;
        mRate = -actualMassFlow;
        float exhaustVel = baseParams.specificImpulse * 9.80665f;
        float thrustMag = actualMassFlow * exhaustVel;
        Vector3 localUp = Vector3RotateByQuaternion({0.0f, 1.0f, 0.0f}, baseParams.orientation);
        netForce = Vector3Add(netForce, Vector3Scale(localUp, thrustMag));
    }

    // Apply Translational RCS Force
    if (state.mass > baseParams.dryMass) {
        Vector3 rcsForceLocal = Vector3Scale(baseParams.controlInput, baseParams.rcsPower);
        Vector3 rcsForceWorld = Vector3RotateByQuaternion(rcsForceLocal, baseParams.orientation);
        netForce = Vector3Add(netForce, rcsForceWorld);
    }

    // Final Integration
    Derivative output;
    output.vel = state.vel; // Derivative of position is velocity
    // Derivative of velocity is acceleration (netForce / mass)
    output.accel = { netForce.x / state.mass, netForce.y / state.mass, netForce.z / state.mass };
    output.massRate = mRate; // Derivative of mass is mass rate
    return output;
}

void UpdatePhysics(Rocket& rocket, float deltaTime, float totalTime) {
    // Dynamic CoM & Moment of Inertia
    float fillRatio = rocket.maxFuelMass > 0.0f ? rocket.fuelMass / rocket.maxFuelMass : 0.0f;
    float fuelHeight = (rocket.height * 0.6f) * fillRatio;
    
    float comDryY = 0.0f;
    float comFuelY = (-rocket.height * 0.5f) + (fuelHeight * 0.5f);
    
    float totalMass = rocket.dryMass + rocket.fuelMass;
    if (totalMass > 0.0f) {
        rocket.centerOfMassY = ((rocket.dryMass * comDryY) + (rocket.fuelMass * comFuelY)) / totalMass;
    } else {
        rocket.centerOfMassY = 0.0f;
    }
    
    float iDryCm = (1.0f / 12.0f) * rocket.dryMass * (rocket.height * rocket.height);
    float distDry = rocket.centerOfMassY - comDryY;
    float iDry = iDryCm + (rocket.dryMass * distDry * distDry);
    
    float iFuelCm = (1.0f / 12.0f) * rocket.fuelMass * (fuelHeight * fuelHeight);
    float distFuel = rocket.centerOfMassY - comFuelY;
    float iFuel = iFuelCm + (rocket.fuelMass * distFuel * distFuel);
    
    rocket.momentOfInertia = fmax(iDry + iFuel, 1000.0f);

    // Check for user input
    if (IsKeyDown(KEY_LEFT_SHIFT)) rocket.throttle += 0.5f * deltaTime; // Ramp up
    if (IsKeyDown(KEY_LEFT_CONTROL)) rocket.throttle -= 0.5f * deltaTime; // Ramp down
    if (IsKeyDown(KEY_SPACE)) rocket.throttle = 1.0f; // Max throttle panic button
    rocket.throttle = Clamp(rocket.throttle, 0.0f, 1.0f);
    if (rocket.fuelMass <= 0.0f) rocket.throttle = 0.0f;

    rocket.controlInput = { 0.0f, 0.0f, 0.0f }; // Reset translational RCS every frame

    // --- Auto-Landing (Suicide Burn) Sequence ---
    if (rocket.autoLandEnabled && rocket.fuelMass > 0.0f) {
        double R_earth = 6371000.0;
        double earth_y = -R_earth;
        double rx = (double)rocket.position.x;
        double ry = (double)rocket.position.y - earth_y;
        double rz = (double)rocket.position.z;
        double r_mag = std::sqrt(rx * rx + ry * ry + rz * rz);
        
        // Altitude of the lowest point of the engine bell
        float currentAlt = (float)(r_mag - R_earth) + rocket.modelBottomOffset;

        // Physical Orientation Check
        Vector3 rocketUp = Vector3RotateByQuaternion({0.0f, 1.0f, 0.0f}, rocket.orientation);
        float actualCos = fmaxf(rocketUp.y, 0.1f); // Actual thrust efficiency based on tilt
        
        // Priority 1 (Attitude): Strictly keep the nose pointed World-UP
        rocket.targetOrientation = QuaternionFromVector3ToVector3({0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
        rocket.autoPilotEnabled = true;

        // Priority 2 & 3: Lateral Drift vs Emergency Leveling
        if (rocketUp.y < 0.96f) {
            rocket.controlInput = {0.0f, 0.0f, 0.0f}; // Emergency Leveling takes priority
        } else {
            Vector3 horizontalVel = {rocket.velocity.x, 0.0f, rocket.velocity.z};
            if (Vector3LengthSqr(horizontalVel) > 0.01f) {
                Vector3 driftDir = Vector3Normalize(Vector3Negate(horizontalVel));
                rocket.controlInput = Vector3RotateByQuaternion(driftDir, QuaternionInvert(rocket.orientation));
            }
        }

        // Throttle Control (The Suicide Burn)
        float maxThrust = rocket.massFlowRate * (rocket.specificImpulse * 9.80665f);
        float currentMass = rocket.dryMass + rocket.fuelMass;

        // Maximum Potential Deceleration
        float a_max = ((maxThrust * actualCos) / currentMass) - 9.81f;
        
        // The Deceleration Ratio
        float verticalSpeed = std::abs(rocket.velocity.y);
        float stopDist = (verticalSpeed * verticalSpeed) / (2.0f * fmaxf(a_max, 0.1f));

        if (currentAlt < 0.5f && std::abs(rocket.velocity.y) < 0.5f) {
            // Shutdown
            rocket.throttle = 0.0f;
            rocket.autoLandEnabled = false;
            rocket.autoPilotEnabled = false;
        } else if (currentAlt < 5.0f) {
            // Final Touchdown Lock
            float hoverThrottle = (currentMass * 9.81f) / (maxThrust * actualCos);
            rocket.throttle = Clamp(hoverThrottle + (-1.5f - rocket.velocity.y) * 0.2f, 0.0f, 1.0f);
        } else {
            // Proportional Throttle (The "Soft" Burn)
            float throttleRequired = (stopDist / fmaxf(currentAlt, 1.0f));
            if (rocket.velocity.y < -1.0f) {
                rocket.throttle = Clamp(throttleRequired, 0.0f, 1.0f);
            } else {
                rocket.throttle = 0.0f;
            }
        }
    }

    // Autopilot & Manual Control
    Vector3 localTorque = { 0.0f, 0.0f, 0.0f };
    bool manualInput = (IsKeyDown(KEY_W) || IsKeyDown(KEY_S) || IsKeyDown(KEY_A) || 
                        IsKeyDown(KEY_D) || IsKeyDown(KEY_Q) || IsKeyDown(KEY_E));

    if (rocket.fuelMass > 0.0f) {
        if (manualInput) {
            rocket.autoPilotEnabled = false;
            rocket.pitchPID.Reset();
            rocket.yawPID.Reset();
            rocket.rollPID.Reset();

            if (IsKeyDown(KEY_W)) localTorque.x -= rocket.rcsPower;
            if (IsKeyDown(KEY_S)) localTorque.x += rocket.rcsPower;
            if (IsKeyDown(KEY_A)) localTorque.z -= rocket.rcsPower;
            if (IsKeyDown(KEY_D)) localTorque.z += rocket.rcsPower;
            if (IsKeyDown(KEY_Q)) localTorque.y += rocket.rcsPower;
            if (IsKeyDown(KEY_E)) localTorque.y -= rocket.rcsPower;
        } else {
            if (!rocket.autoPilotEnabled) {
                rocket.autoPilotEnabled = true;
            }
            
            // Prograde Tracking: Align local Y-axis (Up) with the velocity vector
            float velMag = Vector3Length(rocket.velocity);
            if (velMag > 2.0f) { // Only track if moving fast enough to have a stable vector
                Vector3 prograde = Vector3Scale(rocket.velocity, 1.0f / velMag);
                rocket.targetOrientation = QuaternionFromVector3ToVector3({0.0f, 1.0f, 0.0f}, prograde);
            } else {
                rocket.targetOrientation = rocket.orientation; // Hold steady on the pad
            }

            Quaternion diff = QuaternionMultiply(rocket.targetOrientation, QuaternionInvert(rocket.orientation));
            Vector3 error = QuaternionToEuler(diff);

            // Gain Scheduling: Make the PID more aggressive in thick atmosphere 
            // (simulating aerodynamic control surfaces like grid fins)
            float qMultiplier = 1.0f + (rocket.dynamicPressure / 10000.0f);

            localTorque.x = rocket.pitchPID.Update(error.x, deltaTime) * rocket.rcsPower * qMultiplier;
            localTorque.y = rocket.yawPID.Update(error.y, deltaTime) * rocket.rcsPower * qMultiplier;
            localTorque.z = rocket.rollPID.Update(error.z, deltaTime) * rocket.rcsPower * qMultiplier;
        }
        
        float torqueMag = Vector3Length(localTorque);
        float translationMag = Vector3Length(rocket.controlInput);
        if (torqueMag > 0.0f || translationMag > 0.0f) {
            float rcsConsumptionRate = 0.0002f;
            rocket.fuelMass -= (torqueMag + translationMag * 2.0f) * rcsConsumptionRate * deltaTime;
            if (rocket.fuelMass < 0.0f) rocket.fuelMass = 0.0f;
        }
    }
    rocket.appliedTorque = Vector3RotateByQuaternion(localTorque, rocket.orientation);

    // Aerodynamic Torque (Weather-vaning into the wind)
    double rx_pre = (double)rocket.position.x;
    double ry_pre = (double)rocket.position.y - (-6371000.0);
    double rz_pre = (double)rocket.position.z;
    double r_mag_pre = std::sqrt(rx_pre * rx_pre + ry_pre * ry_pre + rz_pre * rz_pre);
    double trueAlt = r_mag_pre - 6371000.0;

    if (trueAlt < 100000.0) {
        float airDensity = 1.225f * std::exp((float)-trueAlt / 8500.0f);
        static fnl_state noise = fnlCreateState();
        noise.noise_type = FNL_NOISE_OPENSIMPLEX2;
        float jetStreamFactor = std::exp(-std::pow(((float)trueAlt - 12000.0f) / 4000.0f, 2.0f));
        float windSpeed = 10.0f + (65.0f * jetStreamFactor);
        
        Vector3 wind = {
            fnlGetNoise2D(&noise, rocket.position.y * 0.001f, totalTime * 0.1f) * windSpeed,
            0.0f,
            fnlGetNoise2D(&noise, rocket.position.x * 0.001f, totalTime * 0.1f) * windSpeed
        };
        
        Vector3 relVel = Vector3Subtract(rocket.velocity, wind);
        Vector3 rocketUp = Vector3RotateByQuaternion({0.0f, 1.0f, 0.0f}, rocket.orientation);
        Vector3 axialVel = Vector3Scale(rocketUp, Vector3DotProduct(relVel, rocketUp));
        Vector3 lateralVel = Vector3Subtract(relVel, axialVel);
        
        float lateralSpeedSq = Vector3LengthSqr(lateralVel);
        if (lateralSpeedSq > 0.001f) {
            float sideArea = rocket.height * (rocket.modelRadius * 2.0f);
            float lateralDragMag = 0.5f * airDensity * lateralSpeedSq * rocket.dragCoefficient * sideArea;
            Vector3 lateralDragDir = Vector3Normalize(Vector3Negate(lateralVel));
            Vector3 lateralForce = Vector3Scale(lateralDragDir, lateralDragMag);
            
            // Dynamic Aerodynamic Lever Arm
            float localCopY = -rocket.height * 0.35f;
            Vector3 localLeverArm = { 0.0f, localCopY - rocket.centerOfMassY, 0.0f };
            Vector3 worldLeverArm = Vector3RotateByQuaternion(localLeverArm, rocket.orientation);
            Vector3 aeroTorque = Vector3CrossProduct(worldLeverArm, lateralForce);
            rocket.appliedTorque = Vector3Add(rocket.appliedTorque, aeroTorque);
        }
    }

    // Angular Integration
    Vector3 angularAccel = Vector3Scale(rocket.appliedTorque, 1.0f / rocket.momentOfInertia);
    rocket.angularVelocity = Vector3Add(rocket.angularVelocity, Vector3Scale(angularAccel, deltaTime));
    rocket.angularVelocity = Vector3Scale(rocket.angularVelocity, 0.98f);

    // Quaternion Update (The 6DOF Fix)
    Quaternion pitchRot = QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, rocket.angularVelocity.x * deltaTime);
    Quaternion yawRot = QuaternionFromAxisAngle({0.0f, 0.0f, 1.0f}, rocket.angularVelocity.z * deltaTime);
    Quaternion rollRot = QuaternionFromAxisAngle({0.0f, 1.0f, 0.0f}, rocket.angularVelocity.y * deltaTime);
    
    rocket.orientation = QuaternionMultiply(rocket.orientation, pitchRot);
    rocket.orientation = QuaternionMultiply(rocket.orientation, yawRot);
    rocket.orientation = QuaternionMultiply(rocket.orientation, rollRot);
    rocket.orientation = QuaternionNormalize(rocket.orientation);
    // DeltaV Calculation
    if (rocket.fuelMass > 0.0f) {
        rocket.availableDeltaV = rocket.specificImpulse * 9.80665f * std::log((rocket.dryMass + rocket.fuelMass) / rocket.dryMass);
    } else {
        rocket.availableDeltaV = 0.0f;
    }
    // Fire System Visuals & Thermodynamics
    double currentAirDensity = 1.225 * std::exp(-(double)rocket.position.y / 8500.0);
    float pressureRatio = 1.225f / (float)currentAirDensity;
    rocket.plumeExpansion = Clamp(pressureRatio * 0.1f, 1.0f, 5.0f);
    static fnl_state fireNoise = fnlCreateState();
    fireNoise.noise_type = FNL_NOISE_OPENSIMPLEX2;
    float fireTremor = fnlGetNoise2D(&fireNoise, totalTime * 100.0f, 0.0f);
    float currentMassFlow = rocket.throttle * rocket.massFlowRate;
    float maxCapacity = rocket.massFlowRate > 0.0f ? rocket.massFlowRate : 1.0f;
    rocket.fireIntensity = (rocket.throttle > 0.01f) ? (currentMassFlow / maxCapacity) + (fireTremor * 0.2f) : 0.0f;
    // Setup RK4 Initial State
    State initialState = { rocket.position, rocket.velocity, rocket.dryMass + rocket.fuelMass };
    // Perform RK4 Steps
    Derivative a = Evaluate(rocket, initialState, totalTime, 0.0f, Derivative{{0,0,0}, {0,0,0}, 0.0f});
    Derivative b = Evaluate(rocket, initialState, totalTime, deltaTime * 0.5f, a);
    Derivative c = Evaluate(rocket, initialState, totalTime, deltaTime * 0.5f, b);
    Derivative d = Evaluate(rocket, initialState, totalTime, deltaTime, c);
    // Calculate RK4 Averages
    Vector3 dPos_dt = { (1.0f/6.0f) * (a.vel.x + 2.0f*(b.vel.x + c.vel.x) + d.vel.x),
                        (1.0f/6.0f) * (a.vel.y + 2.0f*(b.vel.y + c.vel.y) + d.vel.y),
                        (1.0f/6.0f) * (a.vel.z + 2.0f*(b.vel.z + c.vel.z) + d.vel.z) };
    Vector3 dVel_dt = { (1.0f/6.0f) * (a.accel.x + 2.0f*(b.accel.x + c.accel.x) + d.accel.x),
                        (1.0f/6.0f) * (a.accel.y + 2.0f*(b.accel.y + c.accel.y) + d.accel.y),
                        (1.0f/6.0f) * (a.accel.z + 2.0f*(b.accel.z + c.accel.z) + d.accel.z) };
    float dm_dt     = (1.0f/6.0f) * (a.massRate + 2.0f*(b.massRate + c.massRate) + d.massRate);
    // Apply Integrated Physics
    rocket.position.x += dPos_dt.x * deltaTime;
    rocket.position.y += dPos_dt.y * deltaTime;
    rocket.position.z += dPos_dt.z * deltaTime;
    rocket.velocity.x += dVel_dt.x * deltaTime;
    rocket.velocity.y += dVel_dt.y * deltaTime;
    rocket.velocity.z += dVel_dt.z * deltaTime;
    rocket.fuelMass += dm_dt * deltaTime;
    if (rocket.fuelMass < 0.0f) rocket.fuelMass = 0.0f;

    double R_earth = 6371000.0;
    double earth_y = -R_earth;
    double rx = (double)rocket.position.x;
    double ry = (double)rocket.position.y - earth_y;
    double rz = (double)rocket.position.z;
    double current_r_mag = std::sqrt(rx * rx + ry * ry + rz * rz);
    
    // Advanced Telemetry Calculation
    rocket.acceleration = dVel_dt; 
    float currentMass = rocket.dryMass + rocket.fuelMass;
    float localG = (float)((6.67430e-11 * global_M_earth) / std::pow(current_r_mag, 2.0));
    float thrust = (rocket.massFlowRate * rocket.throttle) * (rocket.specificImpulse * 9.80665f);
    rocket.twr = (currentMass * localG) > 0.0f ? thrust / (currentMass * localG) : 0.0f;
    rocket.mach = Vector3Length(rocket.velocity) / 343.0f;
    float airDensity = 1.225f * (float)std::exp(-rocket.position.y / 8500.0f);
    rocket.dynamicPressure = 0.5f * airDensity * (float)std::pow(Vector3Length(rocket.velocity), 2.0);

    // Spherical Ground Collision & Bouncing
    if (current_r_mag <= R_earth) {
        Vector3 normal = { (float)(rx / current_r_mag), (float)(ry / current_r_mag), (float)(rz / current_r_mag) };
        double penetration = R_earth - current_r_mag;
        rocket.position = Vector3Add(rocket.position, Vector3Scale(normal, (float)penetration));
        
        float vDotN = Vector3DotProduct(rocket.velocity, normal);
        if (vDotN < 0.0f) {
            Vector3 oldVelocity = rocket.velocity;

            Vector3 vNormal = Vector3Scale(normal, vDotN);
            Vector3 vTangent = Vector3Subtract(rocket.velocity, vNormal);
            
            Vector3 newVNormal = Vector3Scale(vNormal, -rocket.restitution);
            Vector3 newVTangent = Vector3Scale(vTangent, 1.0f - rocket.groundFriction);
            rocket.velocity = Vector3Add(newVNormal, newVTangent);
            
            // Angular Impulse Calculation
            Vector3 deltaVelocity = Vector3Subtract(rocket.velocity, oldVelocity);
            Vector3 impulse = Vector3Scale(deltaVelocity, currentMass);

            Vector3 localBottom = { 0.0f, -rocket.height * 0.5f, 0.0f };
            Vector3 localTop = { 0.0f, rocket.height * 0.5f, 0.0f };
            Vector3 r_bottom = Vector3RotateByQuaternion(localBottom, rocket.orientation);
            Vector3 r_top = Vector3RotateByQuaternion(localTop, rocket.orientation);
            float bottomDot = Vector3DotProduct(r_bottom, normal);
            float topDot = Vector3DotProduct(r_top, normal);
            Vector3 r_impact = (bottomDot < topDot) ? r_bottom : r_top;

            Vector3 angularImpulse = Vector3CrossProduct(r_impact, impulse);
            Vector3 deltaAngularVelocity = Vector3Scale(angularImpulse, 1.0f / rocket.momentOfInertia);
            rocket.angularVelocity = Vector3Add(rocket.angularVelocity, deltaAngularVelocity);

            rocket.angularVelocity = Vector3Scale(rocket.angularVelocity, 0.6f);
            
            if (Vector3Length(rocket.velocity) < 0.5f && Vector3Length(rocket.angularVelocity) < 0.5f) {
                rocket.velocity = {0.0f, 0.0f, 0.0f};
                rocket.angularVelocity = {0.0f, 0.0f, 0.0f};
            }
        }
    }
    // --- Particle System Update ---
    double density = 1.225 * std::exp(-(double)rocket.position.y / 8500.0);
    float expansionFactor = Clamp(1.225f / (float)density, 1.0f, 15.0f);
    Vector3 localExit = { 0.0f, rocket.modelBottomOffset, 0.0f };
    Vector3 worldExitOffset = Vector3RotateByQuaternion(localExit, rocket.orientation);
    Vector3 currentWorldExitPos = Vector3Add(rocket.position, worldExitOffset);
    if (rocket.throttle > 0.05f) {
        Vector3 moveVec = Vector3Subtract(rocket.position, rocket.prevPosition);
        float distanceTraveled = Vector3Length(moveVec);
        
        float spawnStep = rocket.modelRadius * 0.2f; // Overlap tightly
        int particlesToSpawn = (int)(distanceTraveled / spawnStep);
        if (particlesToSpawn < 1) particlesToSpawn = 1; // Ensure minimum 1

        int totalToSpawn = particlesToSpawn;
        int spawnedCount = 0;
        for (int i = 0; i < Rocket::MAX_PARTICLES && particlesToSpawn > 0; i++) {
            if (!rocket.plume[i].active) {
                rocket.plume[i].maxLife = 1.5f * (0.2f + (0.8f * rocket.throttle));
                rocket.plume[i].life = rocket.plume[i].maxLife;
                rocket.plume[i].active = true;
                
                float t = (float)spawnedCount / (float)totalToSpawn;
                
                Vector3 baseInterpPos = Vector3Lerp(rocket.prevPosition, rocket.position, t);
                Vector3 interpPos = Vector3Add(baseInterpPos, worldExitOffset);
                rocket.plume[i].pos = interpPos;
                
                rocket.plume[i].temperature = rocket.combustionTemp;

                Vector3 exhaustDir = Vector3RotateByQuaternion({0.0f, -1.0f, 0.0f}, rocket.orientation);
                float spread = rocket.modelRadius * 6.0f * rocket.throttle;
                Vector3 randomSpread = { (GetRandomValue(-100, 100) / 100.0f) * spread, 0.0f, (GetRandomValue(-100, 100) / 100.0f) * spread };
                randomSpread = Vector3RotateByQuaternion(randomSpread, rocket.orientation);
                
                float downwardSpeed = 100.0f * rocket.throttle;
                rocket.plume[i].vel = Vector3Add(rocket.velocity, Vector3Add(Vector3Scale(exhaustDir, downwardSpeed), randomSpread));
                rocket.plume[i].thrustDir = exhaustDir; // Maintain UI stretch compatibility
                
                particlesToSpawn--;
                spawnedCount++;
            }
        }
    }

    rocket.activeParticleCount = 0;
    for (int i = 0; i < Rocket::MAX_PARTICLES; i++) {
        if (!rocket.plume[i].active) continue;

        rocket.plume[i].life -= deltaTime;

        if (rocket.plume[i].life <= 0.0f) {
            rocket.plume[i].active = false;
            
            // Spawn persistent smoke if dying near the ground
            if (rocket.plume[i].pos.y <= 1.0f && GetRandomValue(1, 10) <= 2) {
                for (int j = 0; j < Rocket::MAX_SMOKE_PARTICLES; j++) {
                    if (!rocket.smokePlume[j].active) {
                        rocket.smokePlume[j].active = true;
                        rocket.smokePlume[j].pos = rocket.plume[i].pos;
                        rocket.smokePlume[j].vel = rocket.plume[i].vel; // Inherit the radial outward splash
                        rocket.smokePlume[j].vel.y = (float)GetRandomValue(10, 30) / 10.0f; // Buoyant upward drift
                        rocket.smokePlume[j].maxLife = (float)GetRandomValue(40, 80) / 10.0f; // Linger for 4 to 8 seconds
                        rocket.smokePlume[j].life = rocket.smokePlume[j].maxLife;
                        rocket.smokePlume[j].size = rocket.plume[i].size * 2.0f; // Start big
                        break;
                    }
                }
            }
        }

        if (rocket.plume[i].active) {
            rocket.activeParticleCount++;
                
            // Integration
            rocket.plume[i].pos.x += rocket.plume[i].vel.x * deltaTime;
            rocket.plume[i].pos.y += rocket.plume[i].vel.y * deltaTime;
            rocket.plume[i].pos.z += rocket.plume[i].vel.z * deltaTime;

            // Temperature and Size (Mach Diamonds)
            float lifeRatio = rocket.plume[i].life / rocket.plume[i].maxLife;
            rocket.plume[i].temperature = rocket.combustionTemp * lifeRatio;
            
            float age = rocket.plume[i].maxLife - rocket.plume[i].life;
            float distFromNozzle = age * rocket.exhaustVelocity;
            float machOscillation = std::sin(distFromNozzle * 0.2f);
            
            float baseSize = rocket.modelRadius * (1.0f + (1.0f - lifeRatio) * expansionFactor * 0.8f);
            rocket.plume[i].size = baseSize * (1.0f + machOscillation * 0.4f);

            // Air Resistance
            float drag = 3.0f;
            rocket.plume[i].vel.x -= rocket.plume[i].vel.x * drag * deltaTime;
            rocket.plume[i].vel.y -= rocket.plume[i].vel.y * drag * deltaTime;
            rocket.plume[i].vel.z -= rocket.plume[i].vel.z * drag * deltaTime;

            // Ground Collision
            if (rocket.plume[i].pos.y <= 0.0f) {
                rocket.plume[i].pos.y = 0.1f; // Slightly above ground to prevent Z-fighting
                rocket.plume[i].vel.y = 0.0f;
                // Simulate high-pressure gas deflecting outward radially
                rocket.plume[i].vel.x *= 5.0f;
                rocket.plume[i].vel.z *= 5.0f;
                // Rapid Dissipation
                rocket.plume[i].life -= deltaTime * 10.0f;
            }
        }
    }

    // --- Smoke System Update ---
    rocket.activeSmokeCount = 0;
    for (int i = 0; i < Rocket::MAX_SMOKE_PARTICLES; i++) {
        if (!rocket.smokePlume[i].active) continue;

        rocket.smokePlume[i].life -= deltaTime;
        if (rocket.smokePlume[i].life <= 0.0f) {
            rocket.smokePlume[i].active = false;
            continue;
        }

        rocket.activeSmokeCount++;

        // Air resistance and thermal buoyancy
        rocket.smokePlume[i].vel.x -= rocket.smokePlume[i].vel.x * 1.2f * deltaTime;
        rocket.smokePlume[i].vel.z -= rocket.smokePlume[i].vel.z * 1.2f * deltaTime;
        rocket.smokePlume[i].vel.y += 2.0f * deltaTime; // Heat rising

        rocket.smokePlume[i].pos.x += rocket.smokePlume[i].vel.x * deltaTime;
        rocket.smokePlume[i].pos.y += rocket.smokePlume[i].vel.y * deltaTime;
        rocket.smokePlume[i].pos.z += rocket.smokePlume[i].vel.z * deltaTime;

        rocket.smokePlume[i].size += 3.0f * deltaTime; // Smoke billows and expands over time
    }

    // Crucial: Update the tracker for the next frame's interpolation
    rocket.prevPosition = rocket.position;
    rocket.lastWorldExitPos = currentWorldExitPos;

    // Virtual Launch Clamps (Pad Stabilization)
    double current_rx = (double)rocket.position.x;
    double current_ry = (double)rocket.position.y - earth_y;
    double current_rz = (double)rocket.position.z;
    current_r_mag = std::sqrt(current_rx * current_rx + current_ry * current_ry + current_rz * current_rz);
    
    Vector3 clampLocalBottom = { 0.0f, -rocket.height * 0.5f, 0.0f };
    Vector3 clampR_bottom = Vector3RotateByQuaternion(clampLocalBottom, rocket.orientation);
    Vector3 clampBottomPos = Vector3Add(rocket.position, clampR_bottom);
    double clamp_b_ry = (double)clampBottomPos.y - earth_y;
    double clamp_b_mag = std::sqrt((double)clampBottomPos.x * (double)clampBottomPos.x + clamp_b_ry * clamp_b_ry + (double)clampBottomPos.z * (double)clampBottomPos.z);
    double trueAltitude = clamp_b_mag - R_earth;

    if (trueAltitude < 2.0) {
        Vector3 normal = { (float)(current_rx / current_r_mag), (float)(current_ry / current_r_mag), (float)(current_rz / current_r_mag) };
        Vector3 localUp = { 0.0f, 1.0f, 0.0f };
        Vector3 currentUp = Vector3RotateByQuaternion(localUp, rocket.orientation);
        float alignment = Vector3DotProduct(currentUp, normal);
        
        if (alignment > 0.85f) {
            rocket.angularVelocity = Vector3Scale(rocket.angularVelocity, 0.1f);
            Quaternion targetRot = QuaternionFromVector3ToVector3(localUp, normal);
            rocket.orientation = QuaternionSlerp(rocket.orientation, targetRot, deltaTime * 5.0f);
        }
    }

    // Telemetry History
    rocket.altitudeHistory.push_back(rocket.position.y);
    rocket.velocityHistory.push_back(Vector3Length(rocket.velocity));
    if (rocket.altitudeHistory.size() > 300) rocket.altitudeHistory.erase(rocket.altitudeHistory.begin());
    if (rocket.velocityHistory.size() > 300) rocket.velocityHistory.erase(rocket.velocityHistory.begin());
}

std::vector<Vector3> CalculateTrajectory(Rocket state, float timeAhead, int steps) {
    std::vector<Vector3> points;
    if (steps <= 0) return points;
    
    float dt = timeAhead / steps;
    
    double G = 6.67430e-11;
    double R_earth = 6371000.0;
    double earth_y = -R_earth;

    for (int i = 0; i < steps; i++) {
        double rx = (double)state.position.x;
        double ry = (double)state.position.y - earth_y;
        double rz = (double)state.position.z;
        double r_mag = std::sqrt(rx * rx + ry * ry + rz * rz);
        
        double gravityMag = (G * global_M_earth) / (r_mag * r_mag);
        
        state.velocity.x += (float)(-rx / r_mag * gravityMag) * dt;
        state.velocity.y += (float)(-ry / r_mag * gravityMag) * dt;
        state.velocity.z += (float)(-rz / r_mag * gravityMag) * dt;

        state.position = Vector3Add(state.position, Vector3Scale(state.velocity, dt));
        points.push_back(state.position);
        
        if (r_mag <= R_earth) break; // Stop predicting if it hits the ground
    }
    
    return points;
}