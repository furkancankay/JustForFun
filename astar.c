/*
 * ============================================================
 *  A* (A-Star) Pathfinding Simulation on a Tile Map
 * ============================================================
 *
 *  How to compile and run (Mac / Linux terminal):
 *      gcc astar.c -o astar
 *      ./astar
 *
 *  What is A*?
 *  -----------
 *  A* finds the shortest path from a START tile to a GOAL tile.
 *  For every tile it keeps three numbers:
 *
 *      g = how many steps it took to get here from the START
 *      h = a GUESS of how many steps are left to the GOAL
 *      f = g + h   (the total estimated cost of a path through this tile)
 *
 *  It keeps two lists:
 *      OPEN   tiles = tiles we found but have NOT checked yet
 *      CLOSED tiles = tiles we have already checked
 *
 *  The main idea, repeated again and again:
 *      1. Pick the OPEN tile with the smallest f  (the most promising one)
 *      2. If it is the GOAL -> we are done!
 *      3. Move it to CLOSED
 *      4. Look at its neighbors (up, down, left, right) and
 *         add them to OPEN, remembering where we came from (the "parent")
 *
 *  When we reach the GOAL, we follow the parents backwards
 *  to draw the final path.
 *
 *  You can change the map below! Use:
 *      '#' = wall     '~' = water (also an obstacle)
 *      '.' = floor    'S' = start     'G' = goal
 * ============================================================
 */

#include <stdio.h>   /* printf */
#include <stdlib.h>  /* abs */
#include <unistd.h>  /* usleep (used to slow down the animation) */

/* ---------- Settings ---------- */

#define ROWS 15
#define COLS 30
#define DELAY_MS 60   /* milliseconds between animation frames */

/* ---------- The tile map ---------- */

char map[ROWS][COLS + 1] = {   /* +1 because every string ends with '\0' */
    "##############################",
    "#S..#....#.........#.....#...#",
    "#...#.##.#..####...#..#..#.#.#",
    "#.###..#.#..#~~#.###..#....#.#",
    "#.....##.#..#~~#......####.#.#",
    "###.#....#..#..####.#.#....#.#",
    "#...#.####..#.....#.#.#.####.#",
    "#.###.#.....####..#.#.#......#",
    "#.....#.###....#..#.#.######.#",
    "#.#####...#.##.#.##.#~~~~..#.#",
    "#.#...###.#..#.#....####.#.#.#",
    "#.#.#.....##.#.####....#.#...#",
    "#...#.###....#....#..#.#.###.#",
    "#####...#..#...##...#...#..G.#",
    "##############################",
};

/* ---------- Information we store for every tile ---------- */

typedef struct {
    int g;          /* steps from START to this tile             */
    int h;          /* guessed steps from this tile to GOAL      */
    int f;          /* g + h                                     */
    int parentRow;  /* the tile we came from (to rebuild path)   */
    int parentCol;
    int isOpen;     /* 1 = found, waiting to be checked          */
    int isClosed;   /* 1 = already checked                       */
} Tile;

Tile tiles[ROWS][COLS];

int startRow, startCol;
int goalRow, goalCol;

/* ---------- Helper functions ---------- */

/*
 * The "guess" (heuristic): Manhattan distance.
 * Count how many steps up/down + left/right to reach the goal,
 * pretending there are no walls.
 */
int heuristic(int row, int col)
{
    return abs(row - goalRow) + abs(col - goalCol);
}

/* Returns 1 if we can stand on this tile, 0 if not. */
int isWalkable(int row, int col)
{
    if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
        return 0;   /* outside the map */
    }
    char c = map[row][col];
    if (c == '#' || c == '~') {
        return 0;   /* wall or water: we cannot walk here */
    }
    return 1;
}

/* Look through the map to find where 'S' and 'G' are. */
void findStartAndGoal(void)
{
    for (int row = 0; row < ROWS; row++) {
        for (int col = 0; col < COLS; col++) {
            if (map[row][col] == 'S') { startRow = row; startCol = col; }
            if (map[row][col] == 'G') { goalRow  = row; goalCol  = col; }
        }
    }
}

/* Draw the whole map with colors. Each tile is 2 characters wide. */
void drawMap(int step, const char *message)
{
    printf("\033[H");   /* move cursor to top-left (so we draw over the old frame) */
    printf("A* Pathfinding  |  step %d\n\n", step);

    for (int row = 0; row < ROWS; row++) {
        for (int col = 0; col < COLS; col++) {
            char c = map[row][col];

            if (c == '#')                    printf("\033[100m  \033[0m");    /* wall: gray block */
            else if (c == '~')               printf("\033[44m~~\033[0m");     /* water: blue block */
            else if (c == 'S')               printf("\033[1;32mS \033[0m");   /* start: green     */
            else if (c == 'G')               printf("\033[1;31mG \033[0m");   /* goal: red        */
            else if (c == '*')               printf("\033[1;33m* \033[0m");   /* path: yellow     */
            else if (tiles[row][col].isClosed) printf("\033[34mx \033[0m");   /* closed: blue     */
            else if (tiles[row][col].isOpen)   printf("\033[36mo \033[0m");   /* open: cyan       */
            else                             printf("\033[90m. \033[0m");     /* floor: dark gray */
        }
        printf("\n");
    }

    printf("\n");
    printf("\033[100m  \033[0m wall   \033[44m~~\033[0m water   ");
    printf("\033[1;32mS\033[0m start   \033[1;31mG\033[0m goal\n");
    printf("\033[36mo\033[0m open (to check)   \033[34mx\033[0m closed (checked)   ");
    printf("\033[1;33m*\033[0m path\n\n");
    printf("%s\033[K\n", message);   /* \033[K clears the rest of the line */
    fflush(stdout);
}

/* ---------- The A* algorithm ---------- */

int main(void)
{
    printf("\033[2J");   /* clear the screen once at the beginning */

    findStartAndGoal();

    /* Step 0: put the START tile in the OPEN list. */
    Tile *start = &tiles[startRow][startCol];
    start->g = 0;
    start->h = heuristic(startRow, startCol);
    start->f = start->g + start->h;
    start->parentRow = -1;   /* -1 means "no parent" */
    start->parentCol = -1;
    start->isOpen = 1;

    /* The 4 directions we can move: up, down, left, right */
    int moveRow[4] = { -1, 1,  0, 0 };
    int moveCol[4] = {  0, 0, -1, 1 };

    int step = 0;
    int foundGoal = 0;

    while (1) {
        /* --- 1. Find the OPEN tile with the smallest f --- */
        int bestRow = -1, bestCol = -1;

        for (int row = 0; row < ROWS; row++) {
            for (int col = 0; col < COLS; col++) {
                if (!tiles[row][col].isOpen) continue;

                if (bestRow == -1) {               /* first open tile we see */
                    bestRow = row; bestCol = col;
                    continue;
                }

                Tile *t    = &tiles[row][col];
                Tile *best = &tiles[bestRow][bestCol];

                /* smaller f wins; if f is equal, the one closer to the goal (smaller h) wins */
                if (t->f < best->f || (t->f == best->f && t->h < best->h)) {
                    bestRow = row; bestCol = col;
                }
            }
        }

        /* No open tiles left? Then there is no way to reach the goal. */
        if (bestRow == -1) {
            break;
        }

        /* --- 2. Did we reach the GOAL? --- */
        if (bestRow == goalRow && bestCol == goalCol) {
            foundGoal = 1;
            break;
        }

        /* --- 3. Move the current tile from OPEN to CLOSED --- */
        Tile *current = &tiles[bestRow][bestCol];
        current->isOpen = 0;
        current->isClosed = 1;

        /* --- 4. Check all 4 neighbors --- */
        for (int i = 0; i < 4; i++) {
            int nRow = bestRow + moveRow[i];
            int nCol = bestCol + moveCol[i];

            if (!isWalkable(nRow, nCol)) continue;         /* wall or outside map */
            if (tiles[nRow][nCol].isClosed) continue;      /* already checked     */

            Tile *neighbor = &tiles[nRow][nCol];
            int newG = current->g + 1;   /* one more step than the current tile */

            /* If this tile is new, OR we found a shorter way to reach it, update it */
            if (!neighbor->isOpen || newG < neighbor->g) {
                neighbor->g = newG;
                neighbor->h = heuristic(nRow, nCol);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parentRow = bestRow;   /* remember where we came from */
                neighbor->parentCol = bestCol;
                neighbor->isOpen = 1;
            }
        }

        step++;
        drawMap(step, "Searching...");
        usleep(DELAY_MS * 1000);
    }

    /* ---------- Show the result ---------- */

    if (!foundGoal) {
        drawMap(step, "No path found! The goal is blocked by walls.");
        return 0;
    }

    /* Walk backwards from the GOAL to the START using the parents,
       and mark every tile on the way with '*'. */
    int row = tiles[goalRow][goalCol].parentRow;
    int col = tiles[goalRow][goalCol].parentCol;

    while (!(row == startRow && col == startCol)) {
        map[row][col] = '*';

        int parentRow = tiles[row][col].parentRow;
        int parentCol = tiles[row][col].parentCol;
        row = parentRow;
        col = parentCol;

        drawMap(step, "Path found! Drawing it from the goal back to the start...");
        usleep(DELAY_MS * 1000);
    }

    char message[100];
    snprintf(message, sizeof(message),
             "Done! Shortest path length: %d steps.", tiles[goalRow][goalCol].g);
    drawMap(step, message);

    return 0;
}
