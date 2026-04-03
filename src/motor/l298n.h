#ifndef L298N_H
#define L298N_H

// ----------------------------------------------------------------
//  Low-level L298N dual H-bridge driver for 4 N20 motors.
//
//  Speed range:  -1000 (full reverse)  to  +1000 (full forward)
//  PWM wrap:     1000 counts  →  direct mapping of speed to duty
// ----------------------------------------------------------------

// Initialise all 4 motor channels (GPIO + PWM).
// Call once at startup before using any other motor function.
void motor_init(void);

// Set individual wheel speeds.
//   fl = front-left,  rl = rear-left
//   fr = front-right, rr = rear-right
// Values are clamped to [-1000, +1000].
void motor_set(int fl, int rl, int fr, int rr);

// Coast stop — PWM duty set to 0 on all channels.
void motor_stop(void);

#endif // L298N_H
