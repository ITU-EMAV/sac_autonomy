#include "A_star.h"

A_star::A_star(uint8_t planner_variant, float step_size)
{
    // std::cout << "A_star created" << std::endl;
    this->planner_variant = planner_variant;
    this->step_size = step_size;
    this->sq2 = sqrt(2);

    this->motions = new float[8][3]{
        {step_size, 0, step_size},
        {0, step_size, step_size},
        {-step_size, 0, step_size},
        {0, -step_size, step_size},
        {step_size, step_size, sq2 * step_size},
        {-step_size, -step_size, sq2 * step_size},
        {-step_size, step_size, sq2 * step_size},
        {step_size, -step_size, sq2 * step_size}};
}

A_star::A_star(uint8_t planner_variant, float linear_velocity, float steering_limit, float steering_resolution, float dt) {} // TODO implement hybrid A*

void A_star::update_og(int8_t *array, int heigth, int width, float res, float center_x, float center_y)
{
    this->heigth = heigth;
    this->width = width;
    this->og = array;
    this->res = res;
    this->center_x = center_x;
    this->center_y = center_y;
    this->max_itr = this->heigth * this->width;
    this->mark = new bool[heigth * width];
    this->cost = new float[heigth * width];

    memset(this->mark, 0, heigth * width * sizeof(bool));
    memset(this->cost, 0, heigth * width * sizeof(float));
}

node_2d *A_star::plan(node_2d &start, node_2d &goal)
{
    // cout << goal.x.fdata << endl;
    float goal_x = round((goal.x.fdata - center_x) / res);
    float goal_y = round((goal.y.fdata - center_y) / res);
    cout << "received goal " << goal.x.fdata << " " << goal.y.fdata << endl;
    node_2d *p;
    uint curr_itr = 0;
    std::priority_queue<node_2d> open;
    open.push(start);

    while (!open.empty())
    {
        if (max_itr < curr_itr)
        {
            cout << "max iteration reached" << endl;
            return nullptr;
        }
        curr_itr++;

        // cout << "iteration " << curr_itr << endl;

        node_2d tmp = open.top();
        node_2d *node = new node_2d(tmp);
        open.pop();
        garbage_collector.push_back(node);

        // cout << "/n" << node->x.fdata << " " << node->y.fdata << endl;

        int grid_x = round(((node->x.fdata) - center_x) / res);
        int grid_y = round(((node->y.fdata) - center_y) / res);
        // float h_cost_ = sqrt((goal.x.fdata - x) * (goal.x.fdata - x) + (goal.y.fdata - y) * (goal.y.fdata - y));

        get_mark(grid_y, grid_x) = true;

        if (goal_x == grid_x && goal_y == grid_y)
        {
            cout << "goal found" << endl;

            return node;
        }

        for (int i = 0; i < 8; i++) // motions
        {

            float x = (node->x.fdata + motions[i][1]);
            float y = (node->y.fdata + motions[i][0]);
            // cout << x << " " << y << endl;

            int grid_x = round((x - center_x) / res);
            int grid_y = round((y - center_y) / res);

            // if out of grid
            if (grid_x >= width || grid_y >= heigth || grid_x < 0 || grid_y < 0)
            {
                // cout << "out of grid" << endl;
                continue;
            }

            // check if cell is in closed list
            if (get_mark(grid_y, grid_x) == true)
            {
                // cout << "occupied" << endl;
                continue;
            }

            // check if cell is occupied
            if (get_og(grid_y, grid_x) < 0 || get_og(grid_y, grid_x) >= 100)
            {
                // get_mark(grid_y, grid_x) = numeric_limits<float>::max();
                // cout << "not empty" << endl;
                continue;
            }

            float g_cost_ = node->cost.g_cost + motions[i][2];
            // euc cost
            float h_cost_ = sqrt((goal.x.fdata - x) * (goal.x.fdata - x) + (goal.y.fdata - y) * (goal.y.fdata - y));

            // if this cell has lower cost
            // cout << get_cost(grid_y, grid_x) << endl;
            if (get_cost(grid_y, grid_x) > g_cost_ + h_cost_ || get_cost(grid_y, grid_x) == 0)
            {
                // cout << grid_x << " " << grid_y << " " << get_cost(grid_y, grid_x) << endl;
                get_cost(grid_y, grid_x) = g_cost_ + h_cost_;

                node_2d *tmp = new node_2d(x, y, 0.0, g_cost_, h_cost_);

                tmp->parent = node;
                // std::cout << tmp->x.fdata << " " << tmp->y.fdata << " " << tmp->cost.g_cost + tmp->cost.h_cost << std::endl;
                // cout << " g_cost " << tmp->cost.g_cost << " h_cost " << tmp->cost.h_cost << endl;
                open.push(*tmp);
                // delete tmp;
                garbage_collector.push_back(tmp);
            }
        }
    }

    std::cout << "not found" << std::endl;
    return nullptr;
}

int8_t &A_star::get_og(int y, int x)
{
    return *((og + x) + y * width);
}

bool &A_star::get_mark(int y, int x)
{
    return *((mark + x) + y * width);
}

float &A_star::get_cost(int y, int x)
{
    return *((cost + x) + y * width);
}

void A_star::clear_garbage()
{
    delete this->mark;
    delete this->cost;
    // delete this->og;
    cout << "garbage collector size " << garbage_collector.size() << endl;
    for (auto &node : garbage_collector)
    {
        delete node;
    }
    garbage_collector.clear();
}