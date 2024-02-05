#ifndef RRT_H
#define RRT_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <ctime>
#include <array>
#include <cstdlib>

struct Point
{
    Point() = default;
    Point(int _x, int _y): x(_x), y(_y)
    {}
    int x;
    int y;
};

struct Node
{
    Node() = default;
    Node(Point _p, int _parent_idx): p(_p), parent_idx(_parent_idx)
    {}
    Point p;
    int parent_idx;
};

class RRT
{
public:
    // member variable declarations
    Point start;
    Point goal;
    int lookahead;
    int h_map;
    int w_map;
    int thres; 
    // constructor declarations
    RRT(const Point _start, const Point _goal,  int _lookahead, const int _map_h, const int _map_w, int thres_to_goal);
    // func declarations
    Point get_sample_point(const std::vector<std::vector<int>>& _map);
    int get_euclidean_dist(const Point& p1, const Point& p2);
    Point get_closest_node(const std::vector<Node>& tree, const Point& sampled_point);
    bool is_collided(const Point& nearest_node, const Point& new_node, const std::vector<std::vector<int>>& map);
    Point get_unit_vector(const Point& p1, const Point& p2);
    bool is_goal(const Point& p);
    Point go_to_selected_point(const Point& start, const Point& end);
    int get_random_no(int min, int max);
    std::vector<std::array<int, 2>> get_waypoints(Node& goal, std::vector<Node>& tree);
};  


#endif RRT_H