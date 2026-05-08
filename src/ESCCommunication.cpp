#include "ESCCommunication.h"
#include "DShotS3.h"

// 1. Define the global object so main.cpp can use it
DShotESC escs; 

// 2. Setup the 4 motors
DShotS3 m1(4, 0); 
DShotS3 m2(5, 1);
DShotS3 m3(6, 2);
DShotS3 m4(7, 3);

void DShotESC::begin() {
    m1.begin(); m2.begin(); m3.begin(); m4.begin();
    disarm();
}

void DShotESC::writeMotors(MotorSignals signals) {
    // Map your PID/Mixer 1000-2000 range to DShot 48-2047
    m1.sendThrottle(constrain(map(signals.m1, 1000, 2000, 48, 2047), 0, 2047));
    m2.sendThrottle(constrain(map(signals.m2, 1000, 2000, 48, 2047), 0, 2047));
    m3.sendThrottle(constrain(map(signals.m3, 1000, 2000, 48, 2047), 0, 2047));
    m4.sendThrottle(constrain(map(signals.m4, 1000, 2000, 48, 2047), 0, 2047));
}

void DShotESC::disarm() {
    m1.sendThrottle(0); m2.sendThrottle(0); m3.sendThrottle(0); m4.sendThrottle(0);
}