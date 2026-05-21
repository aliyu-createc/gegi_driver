//
// Created by createc on 10/09/2020.
//
#include <phds_gegi_driver/phds_gegi_driver.hpp>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phds_gegi_driver_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    phds_gegi_driver::PhdsGegiDriver phds_gegi_driver(n,pn);
    ros::spin();
    return 0;
}