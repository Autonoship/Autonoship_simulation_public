#include "osp_ros_demo/osp_ros_demo.hpp"

#include <iostream>
#include <string>

#include <ros/ros.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "osp_ros_node");
    ros::NodeHandle nh;

    OSP_Bridge bridge;

    bridge.printSimulator();
    // bridge.test();

    ros::ServiceServer service = nh.advertiseService("update_OSP", &OSP_Bridge::updateOSP, &bridge);

    ros::spin();
    /*
    ros::Rate r(1000); // 2000 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    */

    return 0;
}
