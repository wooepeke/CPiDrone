#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "blink.pio.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "MPU6050.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

// Use for debugging
void i2c_scan() {
    for (int addr = 1; addr < 128; addr++) {
        uint8_t result = i2c_write_blocking(I2C_PORT, addr, NULL, 0, false);
        if (result == 0) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
}

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

    // Create the MPU6050 object with the correct I2C port
    MPU6050 mpu(I2C_PORT);

    // After finding the MPU, make sure to configure
    mpu.mpu6050_reset();
    mpu.mpu6050_configure();

    int16_t accel[3], gyro[3], temp;

    while (true) {
        mpu.mpu6050_read_raw(accel, gyro, &temp);

        // Get the accelerations and gyro readings
        const int16_t* accel_data = mpu.getAccel();
        const int16_t* gyro_data = mpu.getGyro();

        // Convert raw accelerometer values to g
        float accel_g[3];
        accel_g[0] = accel_data[0] / ACCEL_SCALE_FACTOR;
        accel_g[1] = accel_data[1] / ACCEL_SCALE_FACTOR;
        accel_g[2] = accel_data[2] / ACCEL_SCALE_FACTOR;
        
        // Convert raw gyroscope values to degrees per second
        float gyro_dps[3];
        gyro_dps[0] = gyro_data[0] / GYRO_SCALE_FACTOR;
        gyro_dps[1] = gyro_data[1] / GYRO_SCALE_FACTOR;
        gyro_dps[2] = gyro_data[2] / GYRO_SCALE_FACTOR;
        
        // Print converted values
        printf("aX = %.2f g | aY = %.2f g | aZ = %.2f g | gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | temp = %.2f°C\n",
            accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], temp / 340.00 + 36.53);

        sleep_ms(500);
    }

    return 0;
}