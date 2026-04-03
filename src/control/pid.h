#ifndef PID_H
#define PID_H

// ----------------------------------------------------------------
//  Generic PID controller.
//
//  Used in this project for:
//    1. Heading control while driving straight (error = target_yaw - actual_yaw)
//    2. Turn control — spin until yaw error is within tolerance
//
//  Anti-windup: the integral term is clamped to ±integral_limit.
//  Output:      clamped to ±output_limit.
// ----------------------------------------------------------------

typedef struct {
    float kp;              // proportional gain
    float ki;              // integral gain
    float kd;              // derivative gain

    float integral;        // accumulated integral (state)
    float prev_error;      // previous error for derivative (state)

    float integral_limit;  // anti-windup clamp on integral accumulator
    float output_limit;    // clamp on the final output value
} pid_t;

// Initialise (or re-initialise) a PID instance with given gains.
void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit);

// Compute PID output.
//   setpoint = desired value
//   measured = current measured value
//   dt       = time elapsed since last call (seconds)
// Returns the control output (clamped to ±output_limit).
float pid_compute(pid_t *pid, float setpoint, float measured, float dt);

// Reset integral and previous-error accumulators (call before each new move).
void pid_reset(pid_t *pid);

#endif // PID_H
