#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>

// ----------------------------------------------------------------
//  MPU6050 6-axis IMU driver  (I2C, gyro-based yaw integration).
//
//  Only the gyroscope Z-axis is used — sufficient for a 2-D maze
//  where we only need to track heading changes.
//
//  The driver performs a 500-sample static calibration on init to
//  null out the gyro Z zero-rate bias; keep the robot perfectly
//  still on a flat surface during the first ~1 s after power-on.
// ----------------------------------------------------------------

// Initialise I2C and configure the sensor.
// Returns true on success, false if the chip is not detected.
bool mpu6050_init(void);

// Zero the accumulated yaw (call before each new run).
void mpu6050_reset_yaw(void);

// Read raw gyroscope Z rate in degrees/second (bias-corrected).
float mpu6050_read_gz_dps(void);

// Integrate gyro Z over dt seconds and accumulate yaw.
// Call this in every control-loop iteration.
void mpu6050_update(float dt);

// Return the current accumulated yaw in degrees, wrapped to (-180, +180].
float mpu6050_get_yaw(void);

#endif // MPU6050_H
