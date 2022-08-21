#include <ros/ros.h>
#include "best_practice_cpp_pkg/BestPractice.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "best-practice-node");
    ros::NodeHandle node_Handle("~");

    best_practice_cpp_pkg::BestPractice best_practice(node_Handle);
    best_practice_cpp_pkg::BestPractice best_practice_copy(best_practice);
    best_practice_cpp_pkg::BestPractice best_practice_assign{best_practice};

    return 0;
}