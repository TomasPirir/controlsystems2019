#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <WiFi.h>
#include "Source.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "constants.h"
#include "Servo_Control.hpp"
#include <vector> // This is going to try and clear the error
#include <iostream>
#include <string.h>
#include "json_helper.hpp"

String makeJsonString(std::vector<String>& keys, std::vector<String>& vals);

void initServer(AsyncWebServer* server, ParamsStruct* params, GPS_Struct* GPS) {
     // Create addresses for network connections
    char * ssid = "SJSURoboticsAP";
    char * password = "cerberus2019";
    IPAddress Ip(192, 168, 10, 50);
    IPAddress Gateway(192, 168, 10, 100);
    IPAddress NMask(255, 255, 255, 0);
    
    // Configure the soft AP
    WiFi.mode(WIFI_AP_STA);    
    WiFi.softAP("GimbalESP32", "testpassword");
    Serial.println();
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Connect to the Rover's AP
    WiFi.config(Ip, Gateway, NMask);
    WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(500);
    //     printf("Connecting to WiFi... ");
    // }
    printf("\nConnected to %s\n", ssid);

    AsyncEventSource events("/events");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "PUT, GET, OPTIONS");

    /* XHR Example.
        - Param "name" is sent from mission control.
        - "name" is copied to the params object
        - params object is then passed to vSayHelloTask - see main.cpp
        - vSayHello task then accesses name directly.

        Note: for ANY parameters you want to use, you must add them to
        the paramsStruct struct located in Source.h first. 
    */

   server->on("/pitch_update", HTTP_POST, [=](AsyncWebServerRequest *request){
        strcpy(params->name, request->arg("name").c_str());
        strcpy(params->mode, request->arg("mode").c_str());
        params->manual_move = request->arg("manual_move").toInt();
        params->gimbal_position = request->arg("gimbal_position").toFloat();

        printf("handle_update endpoint running\n");
        printf("    mode: %s \n", params->mode);
        printf("    manual move: %i \n", params->manual_move);
        printf("    gimbal_position: %f \n", params->gimbal_position);
        request->send(200, "text/plain", "Success");
    }); 

    /*
    server->on("/imu_update", HTTP_POST, [=](AsyncWebServerRequest *request){
        strcpy(params->imu_mode, request->arg("imu_mode").c_str());
        params->accelx = request->arg("accelx").toFloat();
        params->accely = request->arg("accely").toFloat();
        params->accelz = request->arg("accelz").toFloat();
        params->gyrox = request->arg("gyrox").toFloat();
        params->gyroy = request->arg("gyroy").toFloat();
        params->gyrox = request->arg("gyroz").toFloat();

        printf("handle_update endpoint running\n");
        printf("    imu_mode: %s \n", params->imu_mode);
        printf("    accelx: %f \n", params->accelx);
        printf("    accely: %f \n", params->accely);
        printf("    accelz: %f \n", params->accelz);
        printf("    gyrox: %f \n", params->gyrox);
        printf("    gyroy: %f \n", params->gyroy);
        printf("    gyro: %f \n", params->gyroz);
        request->send(200, "text/plain", "Success");
    }); */

    server->on("/gps", HTTP_POST, [=](AsyncWebServerRequest *request){
        std::vector <String> keys, vals;
        keys.push_back("longitude");
        keys.push_back("latitude");
        keys.push_back("seconds");
        keys.push_back("centiseconds");
        char** gps_vals;
//        itoa(GPS->longitude, gps_vals[0], 10);
//        itoa(GPS->latitude, gps_vals[1], 10);
//        itoa(GPS->seconds, gps_vals[2], 10);
//        itoa(GPS->centiseconds, gps_vals[3], 10);
        vals.push_back(GPS->longitude);
        vals.push_back(GPS->latitude);
        vals.push_back(GPS->seconds);
        vals.push_back(GPS->centiseconds);

        //call the helper
        String json = makeJsonString(keys, vals);

        //send
        request->send(200, "application/json", json);
    });

    //Attach event source to the server.
    server->addHandler(&events);

    //Start server.
    server->begin();
}

bool initEEPROM() {
    bool status = EEPROM.begin(EEPROM_SIZE);
    switch(status)
    {
        case 0:
	    printf("ERROR: EEPROM initialization failure.\n");
	    usleep(1000);
	    break;
	case 1:
	    printf("Successfully initialized EEPROM, size = %d.\n", EEPROM_SIZE);
	    usleep(1000);
	    break;
	default:
	    break;
    }
    return status;    
}

int EEPROMCount(int addr)
{
    int data = EEPROM.read(addr);
    data++;
    EEPROM.write(addr, data);
    EEPROM.commit();
    return data;
}

void initGimbal() {
    Pitch_Servo.InitServo(PITCH_SERVO_PIN, SERVO_CHANNEL, SERVO_TIMER, 
                      SERVO_FREQUENCY, PITCH_SERVO_MIN, PITCH_SERVO_MAX);

    // Pitch_Servo.InitServoMotor(uint32_t pin, uint32_t channel, uint32_t timer, 
    //                         uint32_t frequency, float max, float min, 
    //                         float dead_min, float dead_max);
    printf("Gimbal has been initialized for movement.\n");

}

void initCameraLens() {
    Pitch_Servo.InitServo(LENS_SERVO_PIN, SERVO_CHANNEL, SERVO_TIMER, 
                      SERVO_FREQUENCY, LENS_SERVO_MIN, LENS_SERVO_MAX);
}

void initPower() {
    pinMode(MOSFET_PIN, OUTPUT);
}

void powerGimbal(int mode) {
    if (mode == 0) {
        digitalWrite(MOSFET_PIN, LOW);
    }
    else if (mode == 1) {
        //I am trying to turn the pwm off
        //experiment start
        Pitch_Servo.SetPositionPercent(0);
        printf("This is turning the position to 0.\n");
        vTaskDelay(150);
        vTaskDelay(150);
        digitalWrite(MOSFET_PIN, HIGH);
        vTaskDelay(150);
        //experiment end
    }
}

void centerMovePitch() {
    Pitch_Servo.SetPositionPercent(SERVO_CENTER);
    printf("The camera is now centered.\n");

    vTaskDelay(150);
}

void upMovePitch(int position) {
    Pitch_Servo.SetPositionPercent(position);
    printf("The camera is rotating upwards.\n");

    vTaskDelay(150);
}

void downMovePitch(int position) {
    Pitch_Servo.SetPositionPercent(position);
    printf("The camera is rotating downwards.\n");

    vTaskDelay(150);
}

void stopMovePitch(int position) {
    Pitch_Servo.SetPositionPercent(position);
    printf("The camera stopped moving.\n");

    vTaskDelay(50);
}

void sweepMovePitch() {
    upMovePitch(SERVO_UP);
    centerMovePitch();
    downMovePitch(SERVO_DOWN);
    printf("Sweeping has been enabled.\n");
}

