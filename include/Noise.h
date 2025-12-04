#ifndef NOISE_H
#define NOISE_H

#include <Arduino.h>

class NoiseGenerator {
public:
    NoiseGenerator();

    // Configuration
    void setGyroNoiseAmplitude(float dps);
    void setAccelNoiseAmplitude(float g);
    void setDriftEnabled(bool enabled);

    // Get a random noise value for the current loop
    float getGyroNoise();
    float getAccelNoise();
    
    // Simulates a slow sensor drift over time
    float getDrift();

private:
    float _gyroAmp;
    float _accelAmp;
    bool _driftEnabled;
    float _currentDrift;
};

#endif