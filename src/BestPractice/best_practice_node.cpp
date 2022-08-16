#include <ros/ros.h>
#include "best_practice_cpp_pkg/BestPractice.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "best-practice-node");
    ros::NodeHandle node_handle_cloud("~");
    
    best_practice_cpp_pkg::BestPractice best_practice(node_handle_cloud);

    best_practice_cpp_pkg::BestPractice best_practice_copy(best_practice);

    best_practice_cpp_pkg::BestPractice best_practice_copy_2(best_practice_copy);

    return 0;
}