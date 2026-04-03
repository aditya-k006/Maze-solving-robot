#include "maze/map.h"
#include <string.h>

// ----------------------------------------------------------------
//  Global tables (definitions — declared extern in map.h)
// ----------------------------------------------------------------
const uint8_t WALL_BIT[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

//                              N   E   S    W
const int DX[4]     = {        0,  1,  0,  -1 };
const int DY[4]     = {        1,  0, -1,   0 };
const int OPP_DIR[4]= {  DIR_S, DIR_W, DIR_N, DIR_E };

cell_t maze_map[MAZE_SIZE][MAZE_SIZE];

// ----------------------------------------------------------------
//  Helpers
// ----------------------------------------------------------------
bool map_in_bounds(int x, int y)
{
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

// ----------------------------------------------------------------
//  Public API
// ----------------------------------------------------------------
void map_init(void)
{
    memset(maze_map, 0, sizeof(maze_map));

    // Set outer boundary walls so the flood fill never goes outside.
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze_map[MAZE_SIZE - 1][i].walls |= WALL_N;  // top row    → north wall
        maze_map[0][i].walls             |= WALL_S;  // bottom row → south wall
        maze_map[i][MAZE_SIZE - 1].walls |= WALL_E;  // right col  → east  wall
        maze_map[i][0].walls             |= WALL_W;  // left col   → west  wall
    }
}

void map_set_wall(int x, int y, int dir)
{
    if (!map_in_bounds(x, y)) return;
    maze_map[y][x].walls |= WALL_BIT[dir];

    // Mirror on the neighbouring cell
    int nx = x + DX[dir];
    int ny = y + DY[dir];
    if (map_in_bounds(nx, ny))
        maze_map[ny][nx].walls |= WALL_BIT[OPP_DIR[dir]];
}

void map_clear_wall(int x, int y, int dir)
{
    if (!map_in_bounds(x, y)) return;
    maze_map[y][x].walls &= ~WALL_BIT[dir];

    int nx = x + DX[dir];
    int ny = y + DY[dir];
    if (map_in_bounds(nx, ny))
        maze_map[ny][nx].walls &= ~WALL_BIT[OPP_DIR[dir]];
}

bool map_has_wall(int x, int y, int dir)
{
    if (!map_in_bounds(x, y)) return true;   // out-of-bounds → treat as wall
    return (maze_map[y][x].walls & WALL_BIT[dir]) != 0;
}

void map_set_visited(int x, int y)
{
    if (map_in_bounds(x, y))
        maze_map[y][x].walls |= VISITED;
}

bool map_is_visited(int x, int y)
{
    if (!map_in_bounds(x, y)) return false;
    return (maze_map[y][x].walls & VISITED) != 0;
}
