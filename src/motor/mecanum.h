#ifndef MECANUM_H
#define MECANUM_H

// ----------------------------------------------------------------
//  Mecanum wheel kinematic mixing layer.
//
//  All velocity inputs are normalised floats in the range [-1.0, 1.0].
//
//  Coordinate convention (robot-centric, viewed from above):
//    vx > 0  →  move FORWARD
//    vx < 0  →  move BACKWARD
//    vy > 0  →  strafe RIGHT
//    vy < 0  →  strafe LEFT
//    omega > 0  →  rotate CLOCKWISE  (turn right)
//    omega < 0  →  rotate COUNTER-CLOCKWISE  (turn left)
//
//  The mixed per-wheel speeds are automatically normalised to avoid
//  saturation, then scaled by MAX_SPEED and forwarded to motor_set().
//
//  NOTE: If the robot strafes in the wrong direction, swap the signs
//  of vy in the FL/RR terms (physical wheel orientation may differ).
// ----------------------------------------------------------------

// General drive command.
void mecanum_drive(float vx, float vy, float omega);

// Convenience wrappers (speed in [0.0, 1.0])
void mecanum_forward (float speed);
void mecanum_backward(float speed);
void mecanum_turn_cw (float speed);   // clockwise  (turn right)
void mecanum_turn_ccw(float speed);   // counter-clockwise (turn left)
void mecanum_stop    (void);

#endif // MECANUM_H
