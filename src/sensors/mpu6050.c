#include "sensors/mpu6050.h"
#include "config.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

// ----------------------------------------------------------------
//  MPU6050 register addresses
// ----------------------------------------------------------------
#define REG_CONFIG        0x1A   // DLPF config
#define REG_GYRO_CONFIG   0x1B   // gyro full-scale range
#define REG_ACCEL_CONFIG  0x1C   // accel full-scale range
#define REG_PWR_MGMT_1    0x6B   // power management
#define REG_PWR_MGMT_2    0x6C
#define REG_GYRO_ZOUT_H   0x47   // gyro Z high byte
#define REG_GYRO_ZOUT_L   0x48   // gyro Z low  byte
#define REG_WHO_AM_I      0x75   // should return 0x68

// At ±250 °/s full-scale:  sensitivity = 131 LSB/(°/s)
#define GYRO_SENS         131.0f

// ----------------------------------------------------------------
//  State
// ----------------------------------------------------------------
static float gyro_z_bias = 0.0f;   // zero-rate offset in raw LSB
static float yaw_deg     = 0.0f;   // accumulated heading (degrees)

// ----------------------------------------------------------------
//  Private helpers
// ----------------------------------------------------------------
static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(MPU_I2C, MPU_ADDR, buf, 2, false);
}

static void mpu_read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    // Send register address, then read `len` bytes
    i2c_write_blocking(MPU_I2C, MPU_ADDR, &reg, 1, true);  // no stop
    i2c_read_blocking (MPU_I2C, MPU_ADDR, dst,   len, false);
}

// Read the 16-bit signed gyro Z value from the sensor
static int16_t read_raw_gz(void)
{
    uint8_t buf[2];
    mpu_read_regs(REG_GYRO_ZOUT_H, buf, 2);
    return (int16_t)((uint16_t)(buf[0] << 8) | buf[1]);
}

// Average 500 samples (~1 s) to compute the static bias.
// Robot MUST be stationary during this phase.
static void calibrate(void)
{
    printf("[MPU6050] Calibrating gyro — keep robot still...\n");
    int64_t sum = 0;
    for (int i = 0; i < 500; i++) {
        sum += read_raw_gz();
        sleep_ms(2);
    }
    gyro_z_bias = (float)(sum) / 500.0f;
    printf("[MPU6050] Gyro Z bias = %.2f LSB\n", gyro_z_bias);
}

// ----------------------------------------------------------------
//  Public API
// ----------------------------------------------------------------
bool mpu6050_init(void)
{
    // --- I2C bus setup ---
    i2c_init(MPU_I2C, I2C_SPEED);
    gpio_set_function(MPU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPU_SDA);
    gpio_pull_up(MPU_SCL);
    sleep_ms(100);  // let the sensor power up

    // --- Verify chip identity ---
    uint8_t who;
    mpu_read_regs(REG_WHO_AM_I, &who, 1);
    if (who != 0x68) {
        printf("[MPU6050] ERROR: WHO_AM_I = 0x%02X (expected 0x68)\n", who);
        return false;
    }
    printf("[MPU6050] Found at address 0x68\n");

    // --- Wake up (clear SLEEP bit) ---
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    sleep_ms(10);

    // --- Digital low-pass filter: 44 Hz bandwidth (smooth, ~10 ms lag) ---
    mpu_write_reg(REG_CONFIG, 0x03);

    // --- Gyro full-scale: ±250 °/s (highest sensitivity) ---
    mpu_write_reg(REG_GYRO_CONFIG, 0x00);

    // --- Accel full-scale: ±2 g (not used for yaw but set cleanly) ---
    mpu_write_reg(REG_ACCEL_CONFIG, 0x00);

    sleep_ms(50);
    calibrate();

    yaw_deg = 0.0f;
    return true;
}

void mpu6050_reset_yaw(void)
{
    yaw_deg = 0.0f;
}

float mpu6050_read_gz_dps(void)
{
    return ((float)read_raw_gz() - gyro_z_bias) / GYRO_SENS;
}

void mpu6050_update(float dt)
{
    float gz = mpu6050_read_gz_dps();
    yaw_deg += gz * dt;

    // Wrap to (-180, +180]
    if (yaw_deg >  180.0f) yaw_deg -= 360.0f;
    if (yaw_deg <= -180.0f) yaw_deg += 360.0f;
}

float mpu6050_get_yaw(void)
{
    return yaw_deg;
}
