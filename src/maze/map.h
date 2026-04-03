#ifndef MAP_H
#define MAP_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ----------------------------------------------------------------
//  Wall bitmask for each cell.
//  One cell stores which of its four sides has a confirmed wall.
// ----------------------------------------------------------------
#define WALL_N   0x01   // north side
#define WALL_E   0x02   // east  side
#define WALL_S   0x04   // south side
#define WALL_W   0x08   // west  side
#define VISITED  0x10   // robot has occupied this cell

// Direction index — keep this consistent with main.c
#define DIR_N  0
#define DIR_E  1
#define DIR_S  2
#define DIR_W  3

// Wall bit indexed by direction
extern const uint8_t WALL_BIT[4];   // [DIR_N]=WALL_N, etc.

// Signed delta for moving one cell in a direction
extern const int DX[4];             // [DIR_E]=+1, [DIR_W]=-1, others 0
extern const int DY[4];             // [DIR_N]=+1, [DIR_S]=-1, others 0

// Opposite direction index
extern const int OPP_DIR[4];        // [DIR_N]=DIR_S, etc.

// ----------------------------------------------------------------
//  The global maze grid.
//  maze_map[y][x].walls  is a bitmask of confirmed walls.
// ----------------------------------------------------------------
typedef struct { uint8_t walls; } cell_t;

extern cell_t maze_map[MAZE_SIZE][MAZE_SIZE];

// ----------------------------------------------------------------
//  API
// ----------------------------------------------------------------

// Reset the grid: outer boundary walls set, all inner walls clear,
// all cells unvisited.
void map_init(void);

// Mark a wall on cell (x, y) AND on the mirrored side of the
// neighbouring cell (so the map stays consistent).
void map_set_wall   (int x, int y, int dir);

// Clear a wall on cell (x, y) and its neighbour.
void map_clear_wall (int x, int y, int dir);

// Returns true if cell (x, y) has a wall on side `dir`.
bool map_has_wall   (int x, int y, int dir);

// Mark cell (x, y) as visited.
void map_set_visited(int x, int y);

// Returns true if cell (x, y) has been visited.
bool map_is_visited (int x, int y);

// Returns true if (x, y) is inside the maze grid.
bool map_in_bounds  (int x, int y);

#endif // MAP_H
