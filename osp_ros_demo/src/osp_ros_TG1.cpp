#include "osp_ros_demo/osp_ros_demo.hpp"

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <osp_ros_demo/updateState_TG1.h>

// the reference id of simulators and variables
int ship_dynamics_id, X_pos_id, Y_pos_id, psi_id, r_id, u_id, v_id;
int propeller_id, rpm_id;
int autopilot_id, target_course_id, local_waypoint_x_id, local_waypoint_y_id, global_waypoint_x_id, global_waypoint_y_id;
int k_id, D_id, w_id;
double k, D, w;

class OSP_Bridge_TG1 : public OSP_Bridge {
protected:
    double rpm;
    double local_waypoint_x, local_waypoint_y, global_waypoint_x, global_waypoint_y;
    
public:
    OSP_Bridge_TG1() : OSP_Bridge() {}
    OSP_Bridge_TG1(std::string p) : OSP_Bridge(p) {}
    
    bool update_secondOrder(double k, double D, double w)
    {
        manipulator->override_real_variable(autopilot_id, k_id, k);
        manipulator->override_real_variable(autopilot_id, D_id, D);
        manipulator->override_real_variable(autopilot_id, w_id, w);
        
        return true;
    }
    
    bool updateOSP(osp_ros_demo::updateState_TG1::Request &req, osp_ros_demo::updateState_TG1::Response &res)
    {
        /*
        Simulator: Hull, Index: 4
        Simulator: Rudder, Index: 3
        Simulator: FPP, Index: 2
        Simulator: HCS, Index: 1
        Simulator: WindandWave, Index: 0
        */

        // inputs and parameters
        double time = req.time.data;
        rpm = req.rpm.data;
        target_course = req.target_course.data;
        local_waypoint_x = req.local_waypoint_x.data;
        local_waypoint_y = req.local_waypoint_y.data;
        global_waypoint_x = req.global_waypoint_x.data;
        global_waypoint_y = req.global_waypoint_y.data;

        cosim::simulator_index ship_dynamics = ship_dynamics_id;  // in this demo, the index of the ship dynamics is 2, AND CAN BE FOUND IN THE index_maps
        cosim::value_reference var_index[] = {X_pos_id, Y_pos_id, psi_id, r_id, u_id, v_id};  // look at the order of the fmu description, the 27th, 35th, 44th, 45th, 47th, 48th variables are the X_pos, Y_pos, psi, r, u, v (start from 0)
        gsl::span<const cosim::value_reference> variables{var_index};
        double init_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gsl::span<double> values{init_values};

        manipulator->override_real_variable(propeller_id, rpm_id, rpm);
        if (target_course_id >= 0) {
          manipulator->override_real_variable(autopilot_id, target_course_id, target_course);
        } else if (local_waypoint_x_id >= 0 && local_waypoint_y_id >= 0) {
          manipulator->override_real_variable(autopilot_id, local_waypoint_x_id, local_waypoint_x_id);
          manipulator->override_real_variable(autopilot_id, local_waypoint_y_id, local_waypoint_y_id);
        } else if (global_waypoint_x_id >= 0 && global_waypoint_y_id >= 0) {
          manipulator->override_real_variable(autopilot_id, global_waypoint_x_id, global_waypoint_x_id);
          manipulator->override_real_variable(autopilot_id, global_waypoint_y_id, global_waypoint_y_id);
        } else {
          ROS_ERROR("Inputs of autopilot not specified! Please check the launch file!");
        }
        
        pExe->simulate_until(cosim::to_time_point(time)).get();
        // ROS_INFO_STREAM("Simulate to: " << time);

        last_value_obs->get_real(ship_dynamics, variables, values);

        res.x.data = values[0];
        res.y.data = values[1];
        res.psi.data = values[2];
        res.r.data = values[3];
        res.u.data = values[4];
        res.v.data = values[5];
        // ROS_INFO((double) values[0], (double) values[1], (double) values[2]);
        ROS_INFO_STREAM(values[0] << values[1] << values[2]);

        return true;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "osp_ros_node");
    ros::NodeHandle nh;

    std::string path = ros::package::getPath("osp_ros_demo").append("/osp_simulation_model_TG1");
    OSP_Bridge_TG1 bridge(path);
    
    // read the reference id of simulators and variables from the parameters in the launch file
    nh.getParam("TG1_OSP_bridge/ship_dynamics_id", ship_dynamics_id);
    nh.getParam("TG1_OSP_bridge/X_pos_id", X_pos_id);
    nh.getParam("TG1_OSP_bridge/Y_pos_id", Y_pos_id);
    nh.getParam("TG1_OSP_bridge/psi_id", psi_id);
    nh.getParam("TG1_OSP_bridge/r_id", r_id);
    nh.getParam("TG1_OSP_bridge/u_id", u_id);
    nh.getParam("TG1_OSP_bridge/v_id", v_id);

    nh.getParam("TG1_OSP_bridge/propeller_id", propeller_id);
    nh.getParam("TG1_OSP_bridge/rpm_id", rpm_id);

    nh.getParam("TG1_OSP_bridge/autopilot_id", autopilot_id);
    nh.getParam("TG1_OSP_bridge/target_course_id", target_course_id);
    nh.getParam("TG1_OSP_bridge/local_waypoint_x_id", local_waypoint_x_id);
    nh.getParam("TG1_OSP_bridge/local_waypoint_y_id", local_waypoint_y_id);
    nh.getParam("TG1_OSP_bridge/global_waypoint_x_id", global_waypoint_x_id);
    nh.getParam("TG1_OSP_bridge/global_waypoint_y_id", global_waypoint_y_id);
    
    
    if (!(nh.getParam("TG1_OSP_bridge/k", k) && nh.getParam("TG1_OSP_bridge/k_id", k_id))) {
      ROS_ERROR("Parameter 'k' not loaded!");
    }
    if (!(nh.getParam("TG1_OSP_bridge/D", D) && nh.getParam("TG1_OSP_bridge/D_id", D_id))) {
      ROS_ERROR("Parameter 'D' not loaded!");
    }
    if (!(nh.getParam("TG1_OSP_bridge/w", w) && nh.getParam("TG1_OSP_bridge/w_id", w_id))) {
      ROS_ERROR("Parameter 'w' not loaded!");
    }
    bridge.update_secondOrder(k, D, w);

    std::cout << (bridge.getPath()) << std::endl;
    bridge.printSimulator();
    // bridge.test();

    ros::ServiceServer service = nh.advertiseService("update_OSP", &OSP_Bridge_TG1::updateOSP, &bridge);

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
