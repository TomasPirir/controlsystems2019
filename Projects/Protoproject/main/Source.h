#pragma once
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <stdlib.h>
#include <stdio.h>
#include "Arduino.h"
#include "Servo_Control.hpp"
#include "constants.h"
#include "vector.hpp"

#ifdef _cplusplus
extern "C" {
#endif

struct ParamsStruct {
    char name[20]; // test parameter
    char mode[20]; // debug, manual, arm, or power control mode based on "debug", "manual", "arm", "on", or "off" 
    int manual_move; // changes the position of the gimbal manual, up, center, or down based on "0", "1", "2", or "3" respectively
    double gimbal_position = 0; // current y axis value of the gimbal
    char imu_mode[20];
    double accelx = 0;
    double accely = 0;
    double accelz = 0;
    double gyrox = 0;
    double gyroy = 0;
    double gyroz = 0; 
};

struct GPS_Struct {
    char longitude[12];
    char latitude[12];
    char *seconds;
    char *centiseconds;
};

typedef enum CommandMoveMode {
    CENTER = 0, 
    UP = 1, 
    DOWN = 2,
    STOP = 3
};

void initServer(AsyncWebServer* server, ParamsStruct* params, GPS_Struct* GPS);

bool initEEPROM();

int EEPROMCount(int addr);

void hello_world(char* name);

void initGimbal(); // Initialize the servo object of the camera pitch

void initCameraLens(); // Initialize the servo object of the DC Camera Lens

void initPower(); // Initializes the kill switch object

void powerGimbal(int mode); // Sends a 3.3V signal to a MOSFET on the gimbal and Jetson PCB turn off the gimbal

void readIMU(); // Interprets values from the IMU underneath the gimbal

void centerMovePitch(); // Rotates gimbal where the camera aligns all the way to the center

void upMovePitch(int position); // Rotates gimbal to shift the camera upwards

void downMovePitch(int position); // Rotates gimbal to shift the camera downwards

void stopMovePitch(int position); // Keeps the gimbal at the position the user wants

void sweepMovePitch(); // Test function to make sure the gimbal works

#ifdef _cplusplus
}
#endif


