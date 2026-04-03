#include "maze/floodfill.h"
#include "maze/map.h"
#include <string.h>

// ----------------------------------------------------------------
//  Distance array (globally accessible)
// ----------------------------------------------------------------
uint8_t flood[MAZE_SIZE][MAZE_SIZE];

// ----------------------------------------------------------------
//  Goal coordinates (set by floodfill_init)
// ----------------------------------------------------------------
static int goal_x_g = GOAL_X;
static int goal_y_g = GOAL_Y;

// ----------------------------------------------------------------
//  Simple circular BFS queue.
//  Maximum cells = MAZE_SIZE * MAZE_SIZE = 256 for a 16×16 maze.
//  We use a slightly larger power-of-2 buffer to be safe.
// ----------------------------------------------------------------
#define QUEUE_SIZE  512

typedef struct { int8_t x; int8_t y; } qcell_t;

static qcell_t bfs_queue[QUEUE_SIZE];
static int q_head, q_tail;

static void q_reset(void)  { q_head = q_tail = 0; }
static bool q_empty(void)  { return q_head == q_tail; }

static void q_push(int x, int y)
{
    bfs_queue[q_tail].x = (int8_t)x;
    bfs_queue[q_tail].y = (int8_t)y;
    q_tail = (q_tail + 1) % QUEUE_SIZE;
}

static qcell_t q_pop(void)
{
    qcell_t c = bfs_queue[q_head];
    q_head = (q_head + 1) % QUEUE_SIZE;
    return c;
}

// ----------------------------------------------------------------
//  Core BFS flood fill
//
//  Starts from the goal (distance 0) and propagates outward,
//  incrementing the distance by 1 for each wall-free step.
//  Cells blocked by walls or already assigned a shorter distance
//  are skipped.
// ----------------------------------------------------------------
static void run_bfs(void)
{
    // Initialise all cells to infinity
    memset(flood, FLOOD_INFINITY, sizeof(flood));

    q_reset();
    flood[goal_y_g][goal_x_g] = 0;
    q_push(goal_x_g, goal_y_g);

    while (!q_empty()) {
        qcell_t cur = q_pop();
        int x = cur.x;
        int y = cur.y;
        uint8_t next_dist = flood[y][x] + 1u;
        if (next_dist == FLOOD_INFINITY) continue; // would overflow

        for (int d = 0; d < 4; d++) {
            // Skip direction if a wall blocks it
            if (map_has_wall(x, y, d)) continue;

            int nx = x + DX[d];
            int ny = y + DY[d];
            if (!map_in_bounds(nx, ny)) continue;

            if (flood[ny][nx] > next_dist) {
                flood[ny][nx] = next_dist;
                q_push(nx, ny);
            }
        }
    }
}

// ----------------------------------------------------------------
//  Public API
// ----------------------------------------------------------------
void floodfill_init(int goal_x, int goal_y)
{
    goal_x_g = goal_x;
    goal_y_g = goal_y;
    run_bfs();
}

void floodfill_update(void)
{
    run_bfs();
}

int floodfill_next_dir(int x, int y)
{
    int   best_dir  = -1;
    uint8_t best_val = FLOOD_INFINITY;

    for (int d = 0; d < 4; d++) {
        if (map_has_wall(x, y, d)) continue;

        int nx = x + DX[d];
        int ny = y + DY[d];
        if (!map_in_bounds(nx, ny)) continue;

        if (flood[ny][nx] < best_val) {
            best_val = flood[ny][nx];
            best_dir = d;
        }
    }

    return best_dir;   // -1 if no valid move (goal unreachable)
}
