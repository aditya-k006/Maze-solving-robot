#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  MAZE SETTINGS
// ============================================================
#define MAZE_SIZE          16       // 16 x 16 cell grid
#define CELL_SIZE_MM       180      // physical size of one cell (mm)

#define START_X            0        // robot start column
#define START_Y            0        // robot start row
#define GOAL_X             7        // goal column  (center-left of 16x16)
#define GOAL_Y             7        // goal row

// Time the robot drives forward to cross exactly one cell (ms).
// Tune this after measuring actual speed on your floor surface.
#define CELL_TRAVEL_MS     900

// Yaw angle tolerance for turn completion (degrees)
#define TURN_TOLERANCE_DEG 2.5f

// ============================================================
//  WALL DETECTION
// ============================================================
// If a TFMini reads below this → wall is present
#define WALL_PRESENT_MM    130
// If a TFMini reads above this → passage is clear
#define WALL_CLEAR_MM      250

// ============================================================
//  MOTOR SPEED LIMITS  (PWM counts, range 0-1000)
// ============================================================
#define BASE_SPEED         420     // straight-line drive speed
#define TURN_SPEED         380     // in-place rotation speed
#define MAX_SPEED          1000    // absolute maximum

// ============================================================
//  GPIO PIN ASSIGNMENTS
// ------------------------------------------------------------
//  L298N #1  →  LEFT motors  (FL = front-left, RL = rear-left)
// ============================================================
#define FL_IN1   2
#define FL_IN2   3
#define FL_EN    10      // PWM  — Slice 5 Channel A

#define RL_IN1   4
#define RL_IN2   5
#define RL_EN    11      // PWM  — Slice 5 Channel B

// L298N #2  →  RIGHT motors  (FR = front-right, RR = rear-right)
#define FR_IN1   6
#define FR_IN2   7
#define FR_EN    12      // PWM  — Slice 6 Channel A

#define RR_IN1   8
#define RR_IN2   9
#define RR_EN    13      // PWM  — Slice 6 Channel B

// ============================================================
//  MPU6050  (I2C0)
// ============================================================
#define MPU_SDA     16
#define MPU_SCL     17
#define MPU_I2C     i2c0
#define MPU_ADDR    0x68
#define I2C_SPEED   400000   // 400 kHz Fast Mode

// ============================================================
//  TFMini LiDAR  (UART — 115200 8N1, robot only needs RX)
//  Front  →  Hardware UART0   GP1  (RX)
//  Left   →  Hardware UART1   GP21 (RX)
//  Right  →  PIO soft-UART    GP22 (RX)
// ============================================================
#define TF_FRONT_UART    uart0
#define TF_FRONT_RX_PIN  1

#define TF_LEFT_UART     uart1
#define TF_LEFT_RX_PIN   21

#define TF_RIGHT_PIO_RX  22      // PIO state machine input pin

#define TFMINI_BAUD      115200

#endif // CONFIG_H
