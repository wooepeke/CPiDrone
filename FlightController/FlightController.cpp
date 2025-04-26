#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

#include "blink.pio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

// I2C defines
#define I2C_PORT i2c0
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

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}


int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

// MPU functions
void mpu6050_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, reset, 2, false);
    sleep_ms(200);
    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);
    sleep_ms(200);
}

void mpu6050_configure() {
    // Set accelerometer range
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);

    // Set gyroscope range
    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    // Set sample rate
    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, sample_rate, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    *temp = (buffer[6] << 8) | buffer[7];
    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];

    // Debug print to check raw values
    printf("Raw Accel X: %d, Y: %d, Z: %d\n", accel[0], accel[1], accel[2]);
    printf("Raw Gyro X: %d, Y: %d, Z: %d\n", gyro[0], gyro[1], gyro[2]);
    printf("Raw Temp: %d\n", *temp);
}

// Use for debuggin
void i2c_scan() {
    for (int addr = 1; addr < 128; addr++) {
        uint8_t result = i2c_write_blocking(I2C_PORT, addr, NULL, 0, false);
        if (result == 0) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
}


int main()
{
    stdio_init_all();

    // Wait for USB Serial to be connected
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("USB Serial connected!\n");

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Scanning for I2C
    printf("Scanning I2C bus...\n");
    i2c_scan();

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif

    uint8_t who_am_i = 0;
    uint8_t reg = WHO_AM_I_REG;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);
    printf("MPU6050 WHO_AM_I: 0x%02X\n", who_am_i);

    if (who_am_i != 0x68) {
        printf("MPU6050 not found!\n");
        while (true);
    }

    // After finding the MPU, make sure to configure
    mpu6050_reset();
    mpu6050_configure();


    int16_t accel[3], gyro[3], temp;

    while (true) {
        mpu6050_read_raw(accel, gyro, &temp);

        // Convert raw accelerometer values to g
        float accel_g[3];
        accel_g[0] = accel[0] / ACCEL_SCALE_FACTOR;
        accel_g[1] = accel[1] / ACCEL_SCALE_FACTOR;
        accel_g[2] = accel[2] / ACCEL_SCALE_FACTOR;

        // Convert raw gyroscope values to degrees per second
        float gyro_dps[3];
        gyro_dps[0] = gyro[0] / GYRO_SCALE_FACTOR;
        gyro_dps[1] = gyro[1] / GYRO_SCALE_FACTOR;
        gyro_dps[2] = gyro[2] / GYRO_SCALE_FACTOR;

        // Print converted values
        printf("aX = %.2f g | aY = %.2f g | aZ = %.2f g | gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | temp = %.2f°C\n",
            accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], temp / 340.00 + 36.53);

        sleep_ms(500);
    }

    return 0;
}
