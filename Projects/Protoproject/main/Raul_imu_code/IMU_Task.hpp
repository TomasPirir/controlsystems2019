#ifndef _IMU_TASK_H
#define _IMU_TASK_H

#include "vector.hpp"
#include "Constants_IMU.hpp"

extern "C" void vMPU6050Task(void *pvParameters)
{
    ParamsStruct *params = (ParamsStruct*) pvParameters;
    imu::Vector<3> accel;
    imu::Vector<3> gyro;

    while(1)
    {
	accel = scanAccel(MPU6050_ADDR0);
	gyro  = scanGyro(MPU6050_ADDR0);

        printf("ACCEL...X: %01f\tY: %01f\tZ: %01f\n", accel.x(), accel.y(), accel.z());
	printf("GYRO....X: %01f\tY: %01f\tZ: %01f\n", gyro.x() , gyro.y() , gyro.z());
	vTaskDelay(100);
    }
}

#endif //_IMU_TASK_H
