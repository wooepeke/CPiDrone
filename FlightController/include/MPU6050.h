#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 register addresses
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

// Sensitivity scale factors for different ranges
#define ACCEL_SCALE_FACTOR_2G 16384.0  // for ±2g
#define ACCEL_SCALE_FACTOR_4G 8192.0   // for ±4g
#define ACCEL_SCALE_FACTOR_8G 4096.0   // for ±8g
#define ACCEL_SCALE_FACTOR_16G 2048.0  // for ±16g

#define GYRO_SCALE_FACTOR_250DPS 131.0    // for ±250 degrees per second
#define GYRO_SCALE_FACTOR_500DPS 65.5     // for ±500 degrees per second
#define GYRO_SCALE_FACTOR_1000DPS 32.8    // for ±1000 degrees per second
#define GYRO_SCALE_FACTOR_2000DPS 16.4    // for ±2000 degrees per second

// Select the desired scale factor
#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G  // Change this to the desired accelerometer range
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS // Change this to the desired gyroscope range

// Corresponding configuration values
#define ACCEL_CONFIG_VALUE 0x08  // for ±4g
#define GYRO_CONFIG_VALUE 0x00  // for ±250 degrees per second
#define SAMPLE_RATE_DIV 1  // Sample rate = 1kHz / (1 + 1) = 500Hz

// Constants for angle calculations
#define RAD_TO_DEG 57.295779513082320876798154814105  // 180/PI
#define DEG_TO_RAD 0.01745329251994329576923690768489 // PI/180
#define TAU 0.05  // Complementary filter constant

class MPU6050 {
    private:
        i2c_inst_t *i2c_port;
        uint8_t address;
        int16_t temp;
        int16_t accel[3];
        int16_t gyro[3];
        
        // Variables for angle calculation
        float accel_angle[3];   // Angles calculated from accelerometer
        float gyro_angle[3];    // Angles calculated from gyroscope
        float angle[3];         // Final filtered angles (roll, pitch, yaw)
        float dt;               // Time between readings
        absolute_time_t last_update_time;
        bool first_run;
        bool calc_yaw;          // Whether to calculate yaw (tends to drift)
        
        // Offsets
        float accel_offset[3];  // Accelerometer offsets
        float gyro_offset[3];   // Gyroscope offsets

    public:
        MPU6050();
        MPU6050(i2c_inst_t *i2c_port);
        
        // Basic functions
        void mpu6050_reset();
        void mpu6050_configure();
        void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
        void i2c_scan();
        
        // Getters for raw values
        const int16_t* getAccel() const { return accel; }
        const int16_t* getGyro() const { return gyro; }
        
        // Angle calculation functions
        void calculateAngles();
        void calculateOffsets();
        void enableYawCalculation(bool enable) { calc_yaw = enable; }
        
        // Get calculated angles
        float getRoll() const { return angle[0]; }
        float getPitch() const { return angle[1]; }
        float getYaw() const { return angle[2]; }
        
        // Get all angles at once
        void getAngles(float *roll, float *pitch, float *yaw) {
            *roll = angle[0];
            *pitch = angle[1];
            *yaw = angle[2];
        }
};

#endif // MPU6050_H