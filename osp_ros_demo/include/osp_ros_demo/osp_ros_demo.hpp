#ifndef OSP_ROS_DEMO_HPP
#define OSP_ROS_DEMO_HPP

#include <ros/package.h>

#include <cosim/algorithm/fixed_step_algorithm.hpp>
#include <cosim/execution.hpp>
#include <cosim/fs_portability.hpp>
#include <cosim/manipulator/scenario_manager.hpp>
#include <cosim/manipulator/override_manipulator.hpp>
#include <cosim/observer/file_observer.hpp>
#include <cosim/observer/observer.hpp>
#include <cosim/observer/last_value_observer.hpp>
#include <cosim/orchestration.hpp>
#include <cosim/osp_config_parser.hpp>
#include <cosim/ssp/ssp_loader.hpp>
#include <cosim/time.hpp>

#include <fmilib.h>
#include <osp_ros_demo/updateState.h>
#include <ros/ros.h>

class OSP_Bridge
{
protected:
    std::string path = ros::package::getPath("osp_ros_demo").append("/osp_simulation_model");
    std::shared_ptr<cosim::model_uri_resolver> uriResolver;
    cosim::osp_config config;
    std::shared_ptr<cosim::execution> pExe;
    cosim::entity_index_maps index_maps;
    std::shared_ptr<cosim::last_value_observer> last_value_obs = std::make_shared<cosim::last_value_observer>();
    std::shared_ptr<cosim::override_manipulator> manipulator = std::make_shared<cosim::override_manipulator>();

    double target_speed;
    double target_course;

public:
    OSP_Bridge();
    OSP_Bridge(std::string path);
    bool updateOSP(osp_ros_demo::updateState::Request &req, osp_ros_demo::updateState::Response &res);
    // bool updateOSP(osp_ros_demo::updateState &req, osp_ros_demo::updateState &res);
    std::string getPath() {return path;}
    void setSpeed(float v) {target_speed = v;}
    void setCourse(float course) {target_course = course;}
    void printSimulator();
    void printFunction();
    void printRealTimeFactor();
    bool test();
};

OSP_Bridge::OSP_Bridge()
{
    OSP_Bridge(path);
    /*
    uriResolver = cosim::default_model_uri_resolver();
    config = cosim::load_osp_config(path, *uriResolver);
    ROS_INFO("start time: %f", cosim::to_double_time_point(config.start_time));
    ROS_INFO("Successfully loaded OSP config!");
    ROS_INFO_STREAM("Config path: " << path);

    // construct execution instance
    pExe = std::make_shared<cosim::execution>(config.start_time, std::make_shared<cosim::fixed_step_algorithm>(config.step_size));
    pExe->get_real_time_config()->real_time_factor_target = 100;
    index_maps = cosim::inject_system_structure(*pExe, config.system_structure, config.initial_values);

    // add observer
    last_value_obs = std::make_shared<cosim::last_value_observer>();
    pExe->add_observer(last_value_obs);

    // create a override_manipulator and execute
    manipulator = std::make_shared<cosim::override_manipulator>();
    pExe->add_manipulator(manipulator);
    */
}

OSP_Bridge::OSP_Bridge(std::string p)
{
    path = p;
    uriResolver = cosim::default_model_uri_resolver();
    config = cosim::load_osp_config(path, *uriResolver);
    ROS_INFO("start time: %f", cosim::to_double_time_point(config.start_time));
    ROS_INFO("Successfully loaded OSP config!");
    ROS_INFO_STREAM("Config path: " << path);

    // construct execution instance
    pExe = std::make_shared<cosim::execution>(config.start_time, std::make_shared<cosim::fixed_step_algorithm>(config.step_size));
    pExe->get_real_time_config()->real_time_factor_target = 100;
    index_maps = cosim::inject_system_structure(*pExe, config.system_structure, config.initial_values);

    // add observer
    last_value_obs = std::make_shared<cosim::last_value_observer>();
    pExe->add_observer(last_value_obs);

    // create a override_manipulator and execute
    manipulator = std::make_shared<cosim::override_manipulator>();
    pExe->add_manipulator(manipulator);
}


bool OSP_Bridge::updateOSP(osp_ros_demo::updateState::Request &req, osp_ros_demo::updateState::Response &res)
{
    /*
    Simulator: ship_dynamics, Index: 2
    Simulator: autopilot, Index: 1
    Simulator: thrust_PID, Index: 0
    */

    // inputs and parameters
    double time = req.time.data;
    target_speed = req.target_speed.data;
    target_course = req.target_course.data;

    cosim::simulator_index ship_dynamics = 2;  // in this demo, the index of the ship dynamics is 2, AND CAN BE FOUND IN THE index_maps
    cosim::value_reference var_index[] = {2, 3, 4, 5, 6};  // look at the order of the fmu description, the 2nd, 3rd, 4th, 5th , 6th variables are the r, psi, x, y and v (start from 0)
    gsl::span<const cosim::value_reference> variables{var_index};
    double init_values[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    gsl::span<double> values{init_values};

    manipulator->override_real_variable(0, 1, target_speed);
    manipulator->override_real_variable(1, 1, target_course);

    pExe->simulate_until(cosim::to_time_point(time)).get();
    // ROS_INFO_STREAM("Simulate to: " << time);

    last_value_obs->get_real(ship_dynamics, variables, values);

    res.phi_dot.data = values[0];
    res.phi.data = values[1];
    res.x.data = values[2];
    res.y.data = values[3];
    res.v.data = values[4];
    // ROS_INFO((double) values[0], (double) values[1], (double) values[2]);
    // ROS_INFO_STREAM(values[0] << values[1] << values[2]);

    return true;
}

void OSP_Bridge::printSimulator()
{
    ROS_INFO("Printing simulators ...");
    for (auto &n : index_maps.simulators)
    {
        ROS_INFO_STREAM("Simulator: " << n.first << ", Index: " << n.second);
    }
}

void OSP_Bridge::printFunction()
{
    for (auto &n : index_maps.functions)
    {
        ROS_INFO_STREAM("Function: " << n.first << ", Index: " << n.second);
    }
}

void OSP_Bridge::printRealTimeFactor()
{
    ROS_INFO_STREAM("Doing real time simulation: " << pExe->get_real_time_config()->real_time_simulation );
    ROS_INFO_STREAM("Real time factor target: " << pExe->get_real_time_config()->real_time_factor_target );
    ROS_INFO_STREAM("Average real time factor: " << pExe->get_real_time_metrics()->total_average_real_time_factor );
    // ROS_INFO_STREAM("Measured real time factor: " << pExe->get_measured_real_time_factor() );
}
bool OSP_Bridge::test()
{
    /*
    Simulator: ship_dynamics, Index: 2
    Simulator: autopilot, Index: 1
    Simulator: thrust_PID, Index: 0
    */

    // inputs and parameters
    double time = 10;

    cosim::simulator_index sim_index = 2;  // in this demo, the index of the ship dynamics is 2, AND CAN BE FOUND IN THE index_maps
    cosim::value_reference var_index[] = {2, 3, 4, 5, 6};  // look at the order of the fmu description, the 2nd, 3rd, 4th, 5th , 6th variables are the r, psi, x, y and v (start from 0)
    gsl::span<const cosim::value_reference> variables{var_index};
    double init_values[] = {0.0, 0.0, 0.0, 0.0, 0.0};  // [r, psi, x, y, v]
    gsl::span<double> values{init_values};

    ROS_INFO("start override input 1");
    manipulator->override_real_variable(0, 1, 7.7166);
    ROS_INFO("start override input 2");
    manipulator->override_real_variable(1, 1, 0.5);

    ROS_INFO_STREAM("run simulation to " << time);
    pExe->simulate_until(cosim::to_time_point(time)).get();

    // ROS_INFO("start override outputs");
    // manipulator->override_real_variable(2, 4, 5);
    // pExe->get_simulator(2)->expose_for_setting(cosim::real, 4);
    // manipulator->override_real_variable(2, 4, 5);

    last_value_obs->get_real(sim_index, variables, values);

    ROS_INFO_STREAM("x: " << values[2] << ", y: " << values[3] << ", v: " << values[4]);
    printRealTimeFactor();
    return true;
}

#endif  // OSP_ROS_DEMO_HPP
