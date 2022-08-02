#include "osp_ros_demo/osp_ros_demo.hpp"

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <osp_ros_demo/updateState_TG1.h>

class OSP_Bridge_TG1 : public OSP_Bridge {
protected:
    double rpm;
    
public:
    OSP_Bridge_TG1() : OSP_Bridge() {}
    OSP_Bridge_TG1(std::string p) : OSP_Bridge(p) {}
    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "osp_ros_node");
    ros::NodeHandle nh;

    std::string local_path;
    nh.getParam("print_sim_id/local_path", local_path);

    std::string path = ros::package::getPath("osp_ros_demo").append(local_path);
    OSP_Bridge_TG1 bridge(path);
    bridge.printSimulator();

    return 0;
}
