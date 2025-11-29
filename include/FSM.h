#ifndef FSM_H
    #define FSM_H

    #include <Arduino.h>
    #include <imu.h>
    #include <Input.h>


    #define DEBUG

    #define BUFFER_SIZE 12
    #define stabilizeTime 100 //in microseconds
    #define stage1 1
    #define stage2 2

    #define ThrottleLowerBound 1000
    #define ThrottleUpperBound 2000

    void FSM(uint8_t *stage);
    void Stage1();
    void Stage2();
    void calcThrottle();
    void printThrottle();
    void init_imu(IMU* imu);
    void initMotorThrottle();
    void setThrottle(int16_t T1, int16_t T2, int16_t T3, int16_t T4);
#endif