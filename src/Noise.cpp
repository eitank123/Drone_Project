#include "Noise.h"

NoiseGenerator::NoiseGenerator() {
    _gyroAmp = 0.0;
    _accelAmp = 0.0;
    _driftEnabled = false;
    _currentDrift = 0.0;
}

void NoiseGenerator::setGyroNoiseAmplitude(float dps) {
    _gyroAmp = dps;
}

void NoiseGenerator::setAccelNoiseAmplitude(float g) {
    _accelAmp = g;
}

void NoiseGenerator::setDriftEnabled(bool enabled) {
    _driftEnabled = enabled;
}

float NoiseGenerator::getGyroNoise() {
    if (_gyroAmp <= 0) return 0.0;
    
    // Generate random float between -1.0 and 1.0
    // esp_random() returns uint32_t (0 to 4,294,967,295)
    float r = ((float)esp_random() / 4294967295.0) * 2.0 - 1.0;
    Serial.println(r * _gyroAmp);
    
    return r * _gyroAmp;
}

float NoiseGenerator::getAccelNoise() {
    if (_accelAmp <= 0) return 0.0;
    
    float r = ((float)esp_random() / 4294967295.0) * 2.0 - 1.0;
    return r * _accelAmp;
}

float NoiseGenerator::getDrift() {
    if (!_driftEnabled) return 0.0;

    // Random walk: slightly increment or decrement the current drift
    float step = ((float)esp_random() / 4294967295.0) * 0.02 - 0.01; 
    _currentDrift += step;
    
    // Constrain drift so it doesn't go crazy
    _currentDrift = constrain(_currentDrift, -5.0, 5.0);
    
    return _currentDrift;
}