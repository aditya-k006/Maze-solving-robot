#include "mecanum.h"
#include "l298n.h"
#include "config.h"
#include <math.h>

// ----------------------------------------------------------------
//  Wheel mixing equations for an X-configuration mecanum drive.
//
//  Physical wheel layout (rollers at 45°):
//
//      FL [\]   FR [/]
//      RL [/]   RR [\]
//
//  Equations derived from standard mecanum kinematics:
//      FL =  vx - vy + omega
//      FR =  vx + vy - omega
//      RL =  vx + vy + omega
//      RR =  vx - vy - omega
//
//  If the robot spins instead of strafing, the vy sign convention
//  is flipped — negate vy in the equations and retest.
// ----------------------------------------------------------------

static float fmaxf4(float a, float b, float c, float d)
{
    float m = fabsf(a);
    if (fabsf(b) > m) m = fabsf(b);
    if (fabsf(c) > m) m = fabsf(c);
    if (fabsf(d) > m) m = fabsf(d);
    return m;
}

void mecanum_drive(float vx, float vy, float omega)
{
    // Per-wheel speed (normalised)
    float fl =  vx - vy + omega;
    float fr =  vx + vy - omega;
    float rl =  vx + vy + omega;
    float rr =  vx - vy - omega;

    // If any wheel exceeds ±1.0, scale all down proportionally
    // so the direction of motion is preserved even at full throttle.
    float max = fmaxf4(fl, fr, rl, rr);
    if (max > 1.0f) {
        fl /= max;
        fr /= max;
        rl /= max;
        rr /= max;
    }

    // Scale to motor driver units and apply
    motor_set(
        (int)(fl * MAX_SPEED),
        (int)(rl * MAX_SPEED),
        (int)(fr * MAX_SPEED),
        (int)(rr * MAX_SPEED)
    );
}

void mecanum_forward(float speed)  { mecanum_drive( speed,  0.0f,  0.0f); }
void mecanum_backward(float speed) { mecanum_drive(-speed,  0.0f,  0.0f); }
void mecanum_turn_cw(float speed)  { mecanum_drive( 0.0f,   0.0f,  speed); }
void mecanum_turn_ccw(float speed) { mecanum_drive( 0.0f,   0.0f, -speed); }
void mecanum_stop(void)            { motor_stop(); }
