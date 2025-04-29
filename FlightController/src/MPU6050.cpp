#include "MPU6050.h"

// Initialize the class
MPU6050::MPU6050() : i2c_port(i2c0), address(MPU6050_ADDR) {}

MPU6050::MPU6050(i2c_inst_t *i2c_port)
    : i2c_port(i2c_port), address(MPU6050_ADDR) {}

// MPU functions
void MPU6050::mpu6050_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(i2c_port, address, reset, 2, false);
    sleep_ms(200);
    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(i2c_port, address, wake, 2, false);
    sleep_ms(200);
}

void MPU6050::mpu6050_configure() {
    // Set accelerometer range
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(i2c_port, address, accel_config, 2, false);

    // Set gyroscope range
    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(i2c_port, address, gyro_config, 2, false);

    // Set sample rate
    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(i2c_port, address, sample_rate, 2, false);
}

void MPU6050::mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;
    i2c_write_blocking(i2c_port, address, &reg, 1, true);
    i2c_read_blocking(i2c_port, address, buffer, 14, false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    *temp = (buffer[6] << 8) | buffer[7];
    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];

    // Store the values in the class members for later access
    this->accel[0] = accel[0];
    this->accel[1] = accel[1];
    this->accel[2] = accel[2];
    this->gyro[0] = gyro[0];
    this->gyro[1] = gyro[1];
    this->gyro[2] = gyro[2];
    this->temp = *temp;

    // Debug print to check raw values
    printf("Raw Accel X: %d, Y: %d, Z: %d\n", accel[0], accel[1], accel[2]);
    printf("Raw Gyro X: %d, Y: %d, Z: %d\n", gyro[0], gyro[1], gyro[2]);
    printf("Raw Temp: %d\n", *temp);
}

void MPU6050::i2c_scan() {
    for (int addr = 1; addr < 128; addr++) {
        uint8_t result = i2c_write_blocking(i2c_port, addr, NULL, 0, false);
        if (result == 0) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
}