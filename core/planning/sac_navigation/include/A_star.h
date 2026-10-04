#ifndef __A_STAR_CPP
#define __A_STAR_CPP

#include "Planner.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <limits>
#include <math.h>
using namespace std;

class A_star : Planner
{
public:
    A_star(uint8_t planner_variant, float step_size);                                                                  // constructor for A*
    A_star(uint8_t planner_variant, float linear_velocity, float steering_limit, float steering_resolution, float dt); // constructor for hybrid A*
    void update_og(int8_t *og, int heigth, int width, float res, float center_x, float center_y);
    node_2d *plan(node_2d &start, node_2d &goal);
    int8_t &get_og(int y, int x);
    bool &get_mark(int y, int x);
    float &get_cost(int y, int x);

    void clear_garbage();
    std::vector<node_2d *> garbage_collector;

    float step_size = 0.05;
    float sq2 = 1.41;
    float (*motions)[3];

    // size of occupancy grid
    int heigth;
    int width;
    // origin of grid
    float center_x;
    float center_y;
    // res of og
    float res;
    // pointer of og
    int8_t *og;

    // mark array
    bool *mark;

    float *cost;

    uint max_itr = 30000;

    // hybrid A*
    uint8_t planner_variant; // 0 is A*, 1 is hybrid A*
    float linear_velocity;
    float steering_limit;
    float steering_resolution;
    float dt; // time step for hybrid A*
};

#endif
