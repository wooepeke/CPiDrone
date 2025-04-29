#include "MPU6050.h"

// Initialize the class
MPU6050::MPU6050() : i2c_port(i2c0), address(MPU6050_ADDR) {
    first_run = true;
    calc_yaw = false;
    dt = 0.01; // Initial time delta
    last_update_time = get_absolute_time();
    
    // Initialize angles to zero
    for (int i = 0; i < 3; i++) {
        accel_angle[i] = 0.0f;
        gyro_angle[i] = 0.0f;
        angle[i] = 0.0f;
        accel_offset[i] = 0.0f;
        gyro_offset[i] = 0.0f;
    }
}

MPU6050::MPU6050(i2c_inst_t *i2c_port)
    : i2c_port(i2c_port), address(MPU6050_ADDR) {
    first_run = true;
    calc_yaw = false;
    dt = 0.001; // Initial time delta
    last_update_time = get_absolute_time();
    
    // Initialize angles to zero
    for (int i = 0; i < 3; i++) {
        accel_angle[i] = 0.0f;
        gyro_angle[i] = 0.0f;
        angle[i] = 0.0f;
        accel_offset[i] = 0.0f;
        gyro_offset[i] = 0.0f;
    }
}

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
}

void MPU6050::i2c_scan() {
    for (int addr = 1; addr < 128; addr++) {
        uint8_t result = i2c_write_blocking(i2c_port, addr, NULL, 0, false);
        if (result == 0) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
}

void MPU6050::calculateOffsets() {
    printf("Calculating offsets. Keep the device still...\n");
    
    const int num_samples = 10000;
    float gyro_sum[3] = {0, 0, 0};
    float accel_sum[3] = {0, 0, 0};
    
    // Take multiple samples to average out noise
    for (int i = 0; i < num_samples; i++) {
        int16_t a[3], g[3], t;
        mpu6050_read_raw(a, g, &t);
        
        // Add to running sum
        for (int j = 0; j < 3; j++) {
            gyro_sum[j] += g[j];
            accel_sum[j] += a[j];
        }
        
        sleep_ms(1); // Small delay between readings
    }
    
    // Calculate average values
    for (int i = 0; i < 3; i++) {
        gyro_offset[i] = gyro_sum[i] / num_samples;
        accel_offset[i] = accel_sum[i] / num_samples;
    }
    
    // Account for gravity in z-axis (should be ~1g when level)
    accel_offset[2] -= ACCEL_SCALE_FACTOR;
    
    printf("Offsets calculated:\n");
    printf("Gyro: X=%f, Y=%f, Z=%f\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
    printf("Accel: X=%f, Y=%f, Z=%f\n", accel_offset[0], accel_offset[1], accel_offset[2]);
}

void MPU6050::calculateAngles() {
    // Read raw values
    int16_t a[3], g[3], t;
    mpu6050_read_raw(a, g, &t);
    
    // Calculate time elapsed since last update
    absolute_time_t current_time = get_absolute_time();
    dt = absolute_time_diff_us(last_update_time, current_time) / 1000000.0f; // Convert to seconds
    last_update_time = current_time;
    
    // Apply offsets and convert to physical units
    float ax = (a[0] - accel_offset[0]) / ACCEL_SCALE_FACTOR;
    float ay = (a[1] - accel_offset[1]) / ACCEL_SCALE_FACTOR;
    float az = (a[2] - accel_offset[2]) / ACCEL_SCALE_FACTOR;
    
    float gx = (g[0] - gyro_offset[0]) / GYRO_SCALE_FACTOR;
    float gy = (g[1] - gyro_offset[1]) / GYRO_SCALE_FACTOR;
    float gz = (g[2] - gyro_offset[2]) / GYRO_SCALE_FACTOR;
    
    // Calculate roll and pitch from accelerometer (in degrees)
    // Using atan2 for better quadrant handling
    accel_angle[0] = atan2(ay, az) * RAD_TO_DEG; // Roll
    accel_angle[1] = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG; // Pitch
    
    // Integrate gyroscope data
    gyro_angle[0] = angle[0] + gx * dt; // Roll
    gyro_angle[1] = angle[1] + gy * dt; // Pitch
    
    if (calc_yaw) {
        gyro_angle[2] = angle[2] + gz * dt; // Yaw (integration only, no accelerometer correction)
    }
    
    // Initial setup for first run
    if (first_run) {
        gyro_angle[0] = accel_angle[0];
        gyro_angle[1] = accel_angle[1];
        gyro_angle[2] = 0; // Initialize yaw to zero
        first_run = false;
    }
    
    // Complementary filter to combine gyro and accelerometer data
    float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
    float gyro_magnitude = sqrt(gx*gx + gy*gy + gz*gz);
    
    // Only apply complementary filter if accelerometer readings are reasonable
    // (close to 1g and not experiencing high accelerations)
    if (accel_magnitude > 0.85 && accel_magnitude < 1.15 && gyro_magnitude < 1.0) {
        angle[0] = (1.0f - TAU) * gyro_angle[0] + TAU * accel_angle[0];
        angle[1] = (1.0f - TAU) * gyro_angle[1] + TAU * accel_angle[1];
    } else {
        // If accelerometer is unreliable (high acceleration), use only gyro
        angle[0] = gyro_angle[0];
        angle[1] = gyro_angle[1];
    }
    
    // For yaw, we can only use gyro (no accelerometer correction)
    if (calc_yaw) {
        angle[2] = gyro_angle[2];
    } else {
        angle[2] = 0;
    }
    
    // Debug output of angles
    // printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", angle[0], angle[1], angle[2]);
}