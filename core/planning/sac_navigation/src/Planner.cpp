
#include "Planner.h"

costs::costs(float g_cost, float h_cost)
{
    this->g_cost = g_cost;
    this->h_cost = h_cost;
}

bool node_2d::operator<(const node_2d &n) const
{
    return cost.g_cost + cost.h_cost > n.cost.g_cost + n.cost.h_cost;
}
bool node_2d::operator==(const node_2d &n) const
{
    return x.data == n.x.data && y.data == n.y.data;
}

node_2d::node_2d(int x, int y, int yaw)
{
    this->x.data = x;
    this->y.data = y;
    this->yaw.data = yaw;
}
node_2d::node_2d(float x, float y, float yaw)
{
    this->x.fdata = x;
    this->y.fdata = y;
    this->yaw.fdata = yaw;
}

node_2d::node_2d(int x, int y, int yaw, float g_cost, float h_cost)
{
    this->x.data = x;
    this->y.data = y;
    this->yaw.data = yaw;
    this->cost.g_cost = g_cost;
    this->cost.h_cost = h_cost;
}
node_2d::node_2d(float x, float y, float yaw, float g_cost, float h_cost)
{
    this->x.fdata = x;
    this->y.fdata = y;
    this->yaw.fdata = yaw;
    this->cost.g_cost = g_cost;
    this->cost.h_cost = h_cost;
}

node_2d::node_2d(const node_2d &n)
{
    x.data = n.x.data;
    y.data = n.y.data;
    yaw.data = n.yaw.data;
    // child = n.child;
    parent = n.parent;
    cost.g_cost = n.cost.g_cost;
    cost.h_cost = n.cost.h_cost;
}
