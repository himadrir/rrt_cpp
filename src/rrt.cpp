#include "rrt.h"

RRT::RRT(const Point _start, const Point _goal,  int _lookahead, const int _map_h, const int _map_w, int thres_to_goal)
{
    start = _start;
    goal = _goal;
    lookahead = _lookahead;
    h_map = _map_h;
    w_map = _map_w;
    thres = thres_to_goal;
    std::srand(std::time(NULL));
}

Point RRT::get_sample_point(const std::vector<std::vector<int>>& _map)
{
    int distX = get_random_no(0, w_map-1);
    int distY = get_random_no(0, h_map-1);
    Point p(distX, distY);
    return p;
}

int RRT::get_euclidean_dist(const Point& p1, const Point& p2)
{
    return std::round( sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) ) );
}

Point RRT::get_closest_node(const std::vector<Node>& tree, const Point& sampled_point)
{
    size_t tree_size = tree.size();
    int  min_dist = std::numeric_limits<int>::max();
    size_t idx = 0;

    for(size_t i=0; i<tree_size; i++)
    {
        int calc_dist = RRT::get_euclidean_dist(tree[i].p, sampled_point);
        
        if(calc_dist < min_dist)
        {
            min_dist = calc_dist;
            idx = i;
        }
    }

    return tree[idx].p;
}

bool RRT::is_collided(const Point& nearest_node, const Point& new_node, const std::vector<std::vector<int>>& map)
{ 
    int x = (new_node.x - nearest_node.x)/lookahead;
    int y = (new_node.y - nearest_node.y)/lookahead;
  
    int curr_x = nearest_node.x;
    int curr_y = nearest_node.y;
  
    for(int i=0;i<lookahead;i++)
    {
        curr_x += x;
        curr_y += y;
        
        if (map[curr_y][curr_x] == 0)
        {
            return true;
        }
    }

    return false;
}

Point RRT::get_unit_vector(const Point& p1, const Point& p2)
{
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    double mag = get_euclidean_dist(p1, p2);

    double x = dx/mag;
    double y = dy/mag;

    Point p(std::round(x), std::round(y));
    
    return p;
}

bool RRT::is_goal(const Point& p)
{
    if(get_euclidean_dist(goal, p) < thres)
    {
        return true;
    }

    else
    {
        return false;
    }
}

Point RRT::go_to_selected_point(const Point& closest_pt, const Point& sampled_point)
{  
    Point unit_vec = get_unit_vector(closest_pt, sampled_point);
    
    int x = closest_pt.x + lookahead * unit_vec.x;
    int y = closest_pt.y + lookahead * unit_vec.y;
    
    if (x >= w_map)
    {
        x = w_map - 1;
    }
   
    else if (y >= h_map)
    {
        y = h_map - 1;
    }

    Point new_point(x, y);
    return new_point;
}

int RRT::get_random_no(int min, int max)
{
    return std::round(std::rand()%((max - min) + 1));
}

std::vector<std::array<int, 2>> RRT::get_waypoints(Node& goal, std::vector<Node>& tree)
{
    std::vector<std::array<int, 2>> found_path;
    std::array<int, 2> curr;
    Node current_node = goal;

    while(current_node.parent_idx != -1)
    {
        curr = {current_node.p.x, current_node.p.y};
        found_path.emplace_back(curr);
        current_node = tree[current_node.parent_idx];
    }
    found_path.push_back({start.x, start.y});
    std::reverse(found_path.begin(), found_path.end());

    return found_path;
} 