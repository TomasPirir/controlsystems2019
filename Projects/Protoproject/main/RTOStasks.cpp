#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "Source.h"
#include "constants.h"
#include "vector.hpp"
#include "scan_imu.hpp"
#include "HardwareSerial.h"
#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include <string>
#include <Wire.h>

extern "C" void vSayHelloTask(void *pvParameters) {
    ParamsStruct* params = (ParamsStruct*) pvParameters;

    while(1) {
        printf("Hello, %s! \n", params->name);
	    vTaskDelay(500);
    }
}

extern "C" void vGPSTask(void *pvParameters)
{
    GPS_Struct *GPS = (GPS_Struct *)pvParameters;

    TinyGPSPlus gps;
    SoftwareSerial ss(RXPIN, TXPIN);
    Serial.begin(115200);
    ss.begin(9600);

    while(1)
    {
        //printf("%d\n", ss.read());
        while(ss.available() > 0)
        {
            gps.encode(ss.read());
            printf("\nLatitude:\n    ");
            Serial.print(gps.location.lat(), 6);
            double lat = gps.location.lat();
            memcpy(GPS->latitude, std::to_string(lat).c_str(), sizeof(lat)); 
//            memcpy(Params->latitude, &lat, sizeof(lat)); // This triggers Guru Meditation Error (StoreProhibited)

            printf("\nLongitude:\n    ");
            Serial.print(gps.location.lng(), 6);
            double lng = gps.location.lng();
            memcpy(GPS->latitude, std::to_string(lng).c_str(), sizeof(lng));

            printf("\nSeconds:\n    ");
            Serial.printf(gps.time.second(), 6);
            uint8_t sec = gps.time.second();
            memcpy(GPS->seconds, std::to_string(sec).c_str(), sizeof(sec));

            printf("\nCentisecods:\n    ");
            Serial.printff(gps.time.centisecond(), 6);
            uint8_t cent = gps.time.centisecond();
            memcpy(GPS->centiseconds, std::to_string(cent).c_str(), sizeof(cent));
        }
        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
            Serial.println(F("No GPS detected: check wiring."));
            while(true);
        }
        vTaskDelay(5);
    }
}

extern "C" void vPitchTask(void *pvParameters) {
    ParamsStruct* params = (ParamsStruct*) pvParameters;
    
    vTaskDelay(500); // This is a test to see if we can delay the initial pwm that is present so the gimbal can Initialize
    vTaskDelay(500); // This is a test to see if we can delay the initial pwm that is present so the gimbal can Initialize
    vTaskDelay(500); // This is a test to see if we can delay the initial pwm that is present so the gimbal can Initialize
    vTaskDelay(500); // This is a test to see if we can delay the initial pwm that is present so the gimbal can Initialize
    vTaskDelay(500); // This is a test to see if we can delay the initial pwm that is present so the gimbal can Initialize
    
    int pitch_position = 35;
    int powermode = 0;

    vTaskDelay(500);
    initGimbal();
    initCameraLens();
    initPower();
    centerMovePitch();
    powerGimbal(powermode);

    while(1) {
        /*
            Debug mode to troubleshoot the gimbal for any errors. IN PROGRESS.
        */
        if (strcmp(params->mode, "debug") == 0) {
            printf("Debug mode\n");
            printf("----------------------.\n");
            sweepMovePitch();
        }
        /*
            Manual mode to allow mission control user to change camera pitch position
            by different command move modes. 

            Case 0: CENTER mode makes the camera gimbal stay centered
            Case 1: UP mode makes the camera gimbal face upwards
            Case 2: DOWN mode makes the camera gimbal stay downwards
            Case 3: STOP mode makes the camera gimbal stop moving and stay in one position

        */
        else if (strcmp(params->mode, "manual") == 0) {
            printf("Manual mode\n");
            printf("----------------------.\n");
            switch(params->manual_move) {
                case CENTER: // CENTER
                    pitch_position = SERVO_CENTER;

                    centerMovePitch();
                    printf("Pitch Position: %i\n", pitch_position);
                    params->gimbal_position = SERVO_CENTER;
                    break;
                case UP: // UP
                    pitch_position -= SERVO_SHIFT;
                    if (pitch_position <= SERVO_UP) {
                        pitch_position = SERVO_UP;
                    }

                    printf("Pitch Position: %i\n", pitch_position);
                    params->gimbal_position = pitch_position;
                    upMovePitch(pitch_position);
                    break;
                case DOWN: // DOWN
                    pitch_position += SERVO_SHIFT;
                    if (pitch_position >= SERVO_DOWN) {
                        pitch_position = SERVO_DOWN;
                    }

                    printf("Pitch Position: %i\n", pitch_position);
                    params->gimbal_position = pitch_position;
                    downMovePitch(pitch_position);
                    break;
                case STOP: // STOP
                    printf("Pitch Position: %i\n", pitch_position);
                    params->gimbal_position = pitch_position;
                    stopMovePitch(pitch_position);
                default:
                    printf("Awaiting pitch position input from mission control.\n");
                    break;
            }
        }
        /*
            Arm mode forces the camera to stay downwards as the rotunda with the 
            robot arm and mast rotates. 
        */
        else if (strcmp(params->mode, "arm") == 0) {
            printf("Arm mode\n");
            printf("----------------------.\n");
            downMovePitch(SERVO_DOWN);
        }
        /*
            The gimbal switches on for camera movement operation.
        */
        else if (strcmp(params->mode, "on") == 0) {
            printf("The gimbal is on\n");
            printf("----------------------.\n");
            powermode = 1;
            powerGimbal(powermode);
        }
        /*
            The gimbal turns off.
        */
        else if (strcmp(params->mode, "off") == 0) {
            printf("The gimbal is off\n");
            printf("----------------------.\n");
            powermode = 0;
            powerGimbal(powermode);
        }
        else {
            // printf("Mode not valid\n");
            // printf("----------------------.\n");
        }

        // sweepMovePitch();
    }   
}

extern "C" void vMPU6050Task(void *pvParameters) 
{
    ParamsStruct* params = (ParamsStruct*) pvParameters;    
    imu::Vector<3> accel;// I changed the location of// I changed the location of// I changed the location of// I changed the location of the * (Original - ParamsStruct *params = (ParamsStruct*) pvParameters;) the * (Original - ParamsStruct *params = (ParamsStruct*) pvParameters;) the * (Original - ParamsStruct *params = (ParamsStruct*) pvParameters;) the * (Original - ParamsStruct *params = (ParamsStruct*) pvParameters;)
    imu::Vector<3> gyro;

    while(1)
    {
	    accel = scanAccel(MPU6050_ADDR0);
	    gyro  = scanGyro(MPU6050_ADDR0);

        printf("ACCEL...X: %01f\tY: %01f\tZ: %01f\n", accel.x(), accel.y(), accel.z());
	    printf("GYRO....X: %01f\tY: %01f\tZ: %01f\n", gyro.x() , gyro.y() , gyro.z());
        if (strcmp(params->imu_mode, "data") == 0){
            params->accelx = accel.x();
            params->accely = accel.y();
            params->accelz = accel.z();
            params->gyrox = gyro.x();
            params->gyroy = gyro.y();
            params->gyroz = gyro.z();
        }
	    vTaskDelay(100);
    }
}

extern "C" void vCountTask(void *pvParameters)
{
    int count = 0;
    EEPROM.put(BEGINING_ADDR, count);
    EEPROM.commit();

    while(1) {
        count = EEPROMCount(BEGINING_ADDR);
        printf("I have said hello %d times!\n\n\n", count);
        vTaskDelay(500);
    }
}
