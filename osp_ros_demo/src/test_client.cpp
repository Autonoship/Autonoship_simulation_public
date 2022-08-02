#include <osp_ros_demo/updateState_TG1.h>

#include <iostream>
#include <string>

#include <ros/ros.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<osp_ros_demo::updateState_TG1>("update_OSP");

    osp_ros_demo::updateState_TG1 srv;
    srv.request.time.data = 0.01;
    srv.request.rpm.data = 94;
    srv.request.target_course.data = 0.5;

    if (client.call(srv))
    {
        ROS_INFO_STREAM("r: " << srv.response.r.data << ", u: " << srv.response.u.data);
    }

    return 0;
}
