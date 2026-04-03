#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#include "config.h"
#include "motor/l298n.h"
#include "motor/mecanum.h"
#include "sensors/mpu6050.h"
#include "sensors/tfmini.h"
#include "control/pid.h"
#include "maze/map.h"
#include "maze/floodfill.h"

// ----------------------------------------------------------------
//  Robot state
// ----------------------------------------------------------------
static int robot_x       = START_X;
static int robot_y       = START_Y;
static int robot_heading = DIR_N;   // 0=N, 1=E, 2=S, 3=W

// PID instance used for straight-line heading correction
static pid_t heading_pid;

// ----------------------------------------------------------------
//  Utility helpers
// ----------------------------------------------------------------
static float fclampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Return the signed shortest angular difference (target - measured),
// normalised to (-180, +180].
static float angle_error(float target, float measured)
{
    float e = target - measured;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

// ----------------------------------------------------------------
//  Turn right (clockwise) 90 degrees.
//  Tracks cumulative rotation via gyro delta, not absolute angle,
//  so wrap-around in mpu6050_get_yaw() never causes issues.
// ----------------------------------------------------------------
static void turn_right_90(void)
{
    printf("[NAV] Turning right 90\n");
    float prev_yaw = mpu6050_get_yaw();
    float rotated  = 0.0f;
    float spd      = (float)TURN_SPEED / MAX_SPEED;

    mecanum_turn_cw(spd);

    while (rotated < (90.0f - TURN_TOLERANCE_DEG)) {
        sleep_ms(10);
        mpu6050_update(0.01f);

        float curr_yaw = mpu6050_get_yaw();
        float delta    = curr_yaw - prev_yaw;
        // Normalise delta to handle the ±180 wrap
        if (delta < -180.0f) delta += 360.0f;
        if (delta >  180.0f) delta -= 360.0f;

        rotated  += delta;
        prev_yaw  = curr_yaw;
    }
    mecanum_stop();
    sleep_ms(150);  // let the robot settle before next action
    printf("[NAV] Right turn done (actual: %.1f deg)\n", rotated);
}

// ----------------------------------------------------------------
//  Turn left (counter-clockwise) 90 degrees.
// ----------------------------------------------------------------
static void turn_left_90(void)
{
    printf("[NAV] Turning left 90\n");
    float prev_yaw = mpu6050_get_yaw();
    float rotated  = 0.0f;
    float spd      = (float)TURN_SPEED / MAX_SPEED;

    mecanum_turn_ccw(spd);

    while (rotated < (90.0f - TURN_TOLERANCE_DEG)) {
        sleep_ms(10);
        mpu6050_update(0.01f);

        float curr_yaw = mpu6050_get_yaw();
        float delta    = prev_yaw - curr_yaw;   // flipped for CCW
        if (delta < -180.0f) delta += 360.0f;
        if (delta >  180.0f) delta -= 360.0f;

        rotated  += delta;
        prev_yaw  = curr_yaw;
    }
    mecanum_stop();
    sleep_ms(150);
    printf("[NAV] Left turn done (actual: %.1f deg)\n", rotated);
}

// ----------------------------------------------------------------
//  Rotate the robot to face target_dir.
//  Chooses the shortest combination of turns.
// ----------------------------------------------------------------
static void face_direction(int target_dir)
{
    int diff = (target_dir - robot_heading + 4) % 4;

    switch (diff) {
        case 1: turn_right_90();                   break;  // one right
        case 2: turn_right_90(); turn_right_90();  break;  // U-turn
        case 3: turn_left_90();                    break;  // one left
        default: break;                                    // already facing
    }
    robot_heading = target_dir;
}

// ----------------------------------------------------------------
//  Drive the robot forward exactly one cell using timed motion
//  and a PID loop to keep the heading straight.
//
//  PID gains (kp, ki, kd) may need tuning on your actual surface.
//  See README "Calibration" section.
// ----------------------------------------------------------------
static void drive_cell(void)
{
    printf("[NAV] Driving forward 1 cell\n");

    float target_yaw = mpu6050_get_yaw();   // lock this heading
    pid_reset(&heading_pid);

    absolute_time_t start  = get_absolute_time();
    absolute_time_t prev_t = start;
    int64_t travel_us = (int64_t)CELL_TRAVEL_MS * 1000LL;

    while (absolute_time_diff_us(start, get_absolute_time()) < travel_us) {
        absolute_time_t now = get_absolute_time();
        float dt = (float)absolute_time_diff_us(prev_t, now) / 1e6f;
        if (dt < 0.0005f) dt = 0.0005f;    // guard against zero dt
        prev_t = now;

        mpu6050_update(dt);
        float yaw = mpu6050_get_yaw();
        float err = angle_error(target_yaw, yaw);

        // PID converts heading error to an omega (rotation) correction.
        // Positive error = drifted left  → need to correct CW (+omega).
        float omega = pid_compute(&heading_pid, 0.0f, -err, dt);
        omega = fclampf(omega, -0.4f, 0.4f);

        float vx = (float)BASE_SPEED / MAX_SPEED;
        mecanum_drive(vx, 0.0f, omega);

        sleep_ms(10);
    }

    mecanum_stop();
    sleep_ms(100);
}

// ----------------------------------------------------------------
//  Read the three TFMini sensors and mark any newly discovered
//  walls on the current cell.  Returns true if any wall changed
//  (so the caller knows to re-run flood fill).
// ----------------------------------------------------------------
static bool sense_and_update(int x, int y, int heading)
{
    uint16_t front = tfmini_get_distance(TF_FRONT);
    uint16_t left  = tfmini_get_distance(TF_LEFT);
    uint16_t right = tfmini_get_distance(TF_RIGHT);

    printf("[SENSE] F:%4d mm  L:%4d mm  R:%4d mm\n", front, left, right);

    // Map sensor positions to absolute compass directions
    int front_dir = heading;
    int left_dir  = (heading + 3) % 4;   // 90° CCW
    int right_dir = (heading + 1) % 4;   // 90° CW

    bool changed = false;

    // We only ADD walls (never remove confirmed walls) because
    // a sensor can fail to detect a wall but should never create
    // a phantom wall in empty space once confirmed absent.
    if (front > 0 && front < WALL_PRESENT_MM && !map_has_wall(x, y, front_dir)) {
        map_set_wall(x, y, front_dir);
        changed = true;
        printf("[MAP] Wall set: (%d,%d) dir %d (FRONT)\n", x, y, front_dir);
    }
    if (left > 0 && left < WALL_PRESENT_MM && !map_has_wall(x, y, left_dir)) {
        map_set_wall(x, y, left_dir);
        changed = true;
        printf("[MAP] Wall set: (%d,%d) dir %d (LEFT)\n", x, y, left_dir);
    }
    if (right > 0 && right < WALL_PRESENT_MM && !map_has_wall(x, y, right_dir)) {
        map_set_wall(x, y, right_dir);
        changed = true;
        printf("[MAP] Wall set: (%d,%d) dir %d (RIGHT)\n", x, y, right_dir);
    }

    return changed;
}

// ================================================================
//  MAIN
// ================================================================
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);     // give time for USB serial to enumerate

    printf("\n================================\n");
    printf("   Mecanum Maze Robot v1.0\n");
    printf("   Algorithm: Flood Fill\n");
    printf("================================\n\n");

    // ---- Motor driver ----
    printf("[INIT] L298N motor drivers...\n");
    motor_init();

    // ---- IMU ----
    printf("[INIT] MPU6050 IMU...\n");
    if (!mpu6050_init()) {
        printf("[FATAL] MPU6050 not detected on I2C. Check wiring. Halting.\n");
        while (true) tight_loop_contents();
    }

    // ---- TFMini sensors ----
    printf("[INIT] TFMini sensors (Front / Left / Right)...\n");
    tfmini_init_all();
    sleep_ms(500);  // sensors need a moment after power-up

    // ---- Maze data structures ----
    printf("[INIT] Map (16x16, goal at %d,%d)...\n", GOAL_X, GOAL_Y);
    map_init();
    floodfill_init(GOAL_X, GOAL_Y);

    // ---- Heading PID ----
    // Tune kp/ki/kd on a straight run:
    //   kp too high → oscillates  |  kp too low → drifts slowly
    //   kd helps damp oscillation |  ki corrects steady-state drift
    pid_init(&heading_pid,
             /*kp*/ 0.004f,
             /*ki*/ 0.0001f,
             /*kd*/ 0.002f,
             /*integral_limit*/ 0.5f,
             /*output_limit*/   0.4f);

    printf("[INIT] All systems ready.\n");
    printf("[INIT] Starting solve in 3 seconds — place robot at (%d,%d)...\n\n",
           START_X, START_Y);
    sleep_ms(3000);

    // ---- Reset navigation state ----
    robot_x       = START_X;
    robot_y       = START_Y;
    robot_heading = DIR_N;
    mpu6050_reset_yaw();
    map_set_visited(robot_x, robot_y);

    // ================================================================
    //  FLOOD FILL SOLVE LOOP
    //
    //  Each iteration: sense → update map → flood → pick direction
    //                  → turn → drive → advance position
    // ================================================================
    while (!(robot_x == GOAL_X && robot_y == GOAL_Y)) {

        printf("\n[LOOP] Position (%d,%d)  heading=%d  distance-to-goal=%d\n",
               robot_x, robot_y, robot_heading,
               flood[robot_y][robot_x]);

        // 1. Sense walls from the current cell
        bool walls_changed = sense_and_update(robot_x, robot_y, robot_heading);
        if (walls_changed) {
            printf("[FLOOD] New walls found — recomputing distances...\n");
            floodfill_update();
        }

        // 2. Decide next direction (cell neighbour with lowest flood value)
        int next_dir = floodfill_next_dir(robot_x, robot_y);
        if (next_dir < 0) {
            printf("[ERROR] Robot is trapped — no valid move exists!\n");
            printf("        Goal may be unreachable with current wall knowledge.\n");
            break;
        }
        int nx = robot_x + DX[next_dir];
        int ny = robot_y + DY[next_dir];
        printf("[DECIDE] Move dir=%d  next-cell=(%d,%d)  flood=%d\n",
               next_dir, nx, ny, flood[ny][nx]);

        // 3. Rotate to face the chosen direction
        face_direction(next_dir);

        // 4. Drive exactly one cell forward
        drive_cell();

        // 5. Advance logical position
        robot_x += DX[robot_heading];
        robot_y += DY[robot_heading];
        map_set_visited(robot_x, robot_y);

        printf("[POS] Entered cell (%d,%d)\n", robot_x, robot_y);
    }

    // ================================================================
    //  GOAL REACHED
    // ================================================================
    mecanum_stop();
    printf("\n*** GOAL REACHED at (%d,%d)! ***\n\n", robot_x, robot_y);

    // Celebrate: spin in place for 2 seconds
    mecanum_turn_cw(0.4f);
    sleep_ms(2000);
    mecanum_stop();

    // Idle forever
    while (true) tight_loop_contents();
}
