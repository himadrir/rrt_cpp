#include "preprocess.h"
#include "rrt.h"
#include <iostream>
#include <opencv2/core/utils/logger.hpp>
//  1 - free 0 - occupied;

/*
image frame
|------------> x
|
|
|
y
*/
int main(int argc, char const *argv[])
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);

    std::string imagePath="./assets/map.bmp"; //edit with path

    cv::Mat image = preprocess::read_img(imagePath);
    
    if(image.empty())
    {
        std::cout<<"ERROR reading image!"<<std::endl;
        return -1;
    }

    std::vector<std::vector<int>> map;
    map = preprocess::get_thresholded_img(image);
    
    // user-defined values: start loc, goal loc, max desired iterations
    //            x,  y
    Point p_start(50, 50);
    Point p_goal(180, 180);
    int max_iterations = 2000;
    unsigned int lookahead_dist = 10;
    unsigned int thres_to_goal = 10;
    // end of user-defined values: start loc, goal loc, max desired iterations

    RRT rrt(p_start, p_goal, lookahead_dist, map.size(), map[0].size(), thres_to_goal); // create RRT object

    Node start_node(p_start, -1);
    std::vector<Node> tree;
    std::vector<std::array<int, 2>> waypoints;
    Point p;
    Point new_p;
    Point closest_point;
    bool is_goal = false;
    tree.push_back(start_node);
    int iter=0;

    for(;iter<max_iterations; iter++)
    {
        p = rrt.get_sample_point(map);
        closest_point = rrt.get_closest_node(tree, p);
        new_p = rrt.go_to_selected_point(closest_point, p);
        size_t closest_idx = 0;
        bool is_collided = rrt.is_collided(closest_point, new_p, map);

        if(is_collided == true)
        {   
            continue; // look for new point again
        }

        for(size_t i = 0; i<tree.size(); i++)
        {
            if(closest_point.x == tree[i].p.x && closest_point.y == tree[i].p.y)
            {
                
                closest_idx = i;
                break;
            }  
        }

        is_goal = rrt.is_goal(new_p);

        if(is_goal)
        {       
            std::cout<<"goal reached!"<<std::endl;
            Node new_node(new_p, closest_idx);
            tree.push_back(new_node); 
            waypoints = rrt.get_waypoints(new_node, tree);
            break;
        }     
        Node new_node(new_p, closest_idx);
        tree.push_back(new_node);           
    }

    int tree_size = tree.size();    
    
    if(waypoints.size() > 0)
    {
        for(int i=0; i<waypoints.size(); i++)
        {
            std::cout<<"("<<waypoints[i][0]<<","<<waypoints[i][1]<<")->";
        }
        std::cout<<std::endl;
    }

    if(is_goal)
    {
        std::cout<<"Iters to converge: "<<iter<<std::endl;

        cv::Point _start(p_start.x, p_start.y);
        cv::Point _goal(p_goal.x, p_goal.y);
        cv::Mat colorized_image;

        preprocess::add_path_to_image(waypoints, image);

        cv::cvtColor(image, colorized_image, cv::COLOR_GRAY2BGR);
        cv::circle(colorized_image, _start, 5, cv::Scalar(0, 255, 0), -1);
        cv::circle(colorized_image, _goal, 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(colorized_image, _goal, thres_to_goal, cv::Scalar(255, 0, 0), 2);
        cv::imshow("RRT final path, GREEN-start RED-goal", colorized_image);
        
        cv::waitKey(0);  
    }
    
    else
    {
        std::cout<<"Failed to converge after "<<iter<<" iterations!"<<std::endl;
    }

    return 0;
}