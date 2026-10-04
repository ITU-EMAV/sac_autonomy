#ifndef __PLANNER__CPP
#define __PLANNER__CPP

#include <memory>

union mixed_data
{
    int data;
    float fdata;
};

struct costs
{
    float g_cost;
    float h_cost;

    costs(){};
    costs(float g_cots, float h_cost);
};

struct node_2d
{

    mixed_data x;
    mixed_data y;
    mixed_data yaw;

    costs cost;

    node_2d *parent = nullptr;
    // node_2d *child = nullptr;

    node_2d(int x, int y, int yaw);
    node_2d(float x, float y, float yaw);
    node_2d(int x, int y, int yaw, float g_cost, float h_cost);
    node_2d(float x, float y, float yaw, float g_cost, float h_cost);

    node_2d(const node_2d &n);

    bool operator<(const node_2d &) const;
    bool operator==(const node_2d &) const;
};

class Planner
{

public:
    void plan(node_2d &start, node_2d &goal);
};

#endif
