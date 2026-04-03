#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdint.h>
#include "config.h"

// ----------------------------------------------------------------
//  Flood-fill maze solver.
//
//  The distance array flood[y][x] stores the minimum number of
//  cell steps from cell (x,y) to the goal, given the walls
//  currently known in maze_map[][].
//
//  Value 255 means "unreachable" (or not yet reached during BFS).
//
//  Workflow:
//    1. Call floodfill_init() once with the goal coordinates.
//    2. After the robot updates any wall, call floodfill_update()
//       to recompute the distance map.
//    3. Call floodfill_next_dir() at the current cell to get the
//       direction the robot should move next.
// ----------------------------------------------------------------

#define FLOOD_INFINITY  255u

extern uint8_t flood[MAZE_SIZE][MAZE_SIZE];

// Set the goal cell and run the initial flood fill.
void floodfill_init(int goal_x, int goal_y);

// Recompute the entire distance map (call whenever walls change).
void floodfill_update(void);

// Return the direction index (DIR_N/E/S/W) of the open neighbour
// with the lowest flood value from cell (x, y).
// Returns -1 if no valid move is available (robot is trapped).
int floodfill_next_dir(int x, int y);

#endif // FLOODFILL_H
