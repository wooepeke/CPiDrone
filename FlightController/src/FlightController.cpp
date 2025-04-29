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
#include <chrono>  // Include for timing

#include "DroneModel.h"
#include "MPU6050.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

// Define LED blink frequencies for different states
#define CALIBRATION_BLINK_FREQ 3  // slow blinking for calibration
#define RUNNING_BLINK_FREQ 1  // slow blinking for running

// Use for debugging
void i2c_scan() {
    for (int addr = 1; addr < 128; addr++) {
        uint8_t result = i2c_write_blocking(I2C_PORT, addr, NULL, 0, false);
        if (result == 0) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
}

// Function to start blinking the LED at a specified frequency
void start_led_blinking(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    
    // Set the frequency
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
    
    printf("Started blinking pin %d at %d Hz\n", pin, freq);
}

// Function to stop the LED blinking
void stop_led_blinking(PIO pio, uint sm) {
    pio_sm_set_enabled(pio, sm, false);
    printf("Stopped LED blinking\n");
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

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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
    
    // Enable yaw calculation if desired
    mpu.enableYawCalculation(true);
    
    // Set up PIO for LED blinking
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    // Start slow blinking to indicate calibration is in progress
    printf("Starting calibration mode LED indicator...\n");
    start_led_blinking(pio, 0, offset, LED_PIN, CALIBRATION_BLINK_FREQ);
    
    // Calculate offsets (make sure the device is still)
    printf("Calibrating MPU6050... Keep the device still\n");
    mpu.calculateOffsets();
    printf("Calibration complete\n");
    
    // Stop the calibration blinking
    stop_led_blinking(pio, 0);
    
    // Start blinking the led indicating that the drone is ready
    start_led_blinking(pio, 0, offset, LED_PIN, RUNNING_BLINK_FREQ);

    // Initialize drone model
    DroneModel drone;

    // Initialize PWM pins for the four motors
    // These are the GPIO pin numbers, not physical pin numbers
    // Choose GPIO pins that support PWM on the Pico
    uint motor1_pin = 0;  // GPIO 0 (physical pin 1)
    uint motor2_pin = 2;  // GPIO 2 (physical pin 4)
    uint motor3_pin = 4;  // GPIO 4 (physical pin 6)
    uint motor4_pin = 6;  // GPIO 6 (physical pin 9)

    drone.initPWMPins(motor1_pin, motor2_pin, motor3_pin, motor4_pin);

    int16_t accel[3], gyro[3], temp;
    float roll, pitch, yaw;

    while (true) {
        // Start timer to measure loop duration
        auto start_time = std::chrono::high_resolution_clock::now();

        // Read raw values
        mpu.mpu6050_read_raw(accel, gyro, &temp);
        
        // Calculate Euler angles
        mpu.calculateAngles();
        
        // Get the calculated angles
        roll = mpu.getRoll();
        pitch = mpu.getPitch();
        yaw = mpu.getYaw();
        
        // Get the accel and gyro readings
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
        
        // // Print raw values
        // printf("Raw - aX = %.2f g | aY = %.2f g | aZ = %.2f g | gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps\n",
        //     accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2]);
            
        // // Print filtered angles
        // printf("Angles - Roll = %.2f° | Pitch = %.2f° | Yaw = %.2f° | Temp = %.2f°C\n", 
        //     roll, pitch, yaw, temp / 340.00 + 36.53);
        
        gpio_put(LED_PIN, 0);

        // End time measurement
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

        // Print loop duration in milliseconds
        // printf("Loop duration: %.2f ms\n", elapsed.count());
    }

    return 0;
}



// EXAMPLE
// // Optional: Set a custom PWM frequency (default is 25kHz)
// // drone.setPWMFrequency(20000); // 20kHz

// // Main control loop
// while (true) {
//     // Example: Gradually increase all motor speeds
//     for (float speed = 0.0; speed <= 1.0; speed += 0.1) {
//         printf("Setting all motors to speed: %.1f\n", speed);
//         drone.setAllMotorSpeeds(speed, speed, speed, speed);
//         sleep_ms(500);
//     }
    
//     // Example: Run motors at different speeds
//     printf("Setting motors to different speeds\n");
//     drone.setMotorSpeed(0, 0.5);  // 50% power
//     drone.setMotorSpeed(1, 0.6);  // 60% power
//     drone.setMotorSpeed(2, 0.7);  // 70% power
//     drone.setMotorSpeed(3, 0.8);  // 80% power
//     sleep_ms(2000);
    
//     // Stop all motors
//     printf("Stopping all motors\n");
//     drone.stopAllMotors();
//     sleep_ms(2000);
// }