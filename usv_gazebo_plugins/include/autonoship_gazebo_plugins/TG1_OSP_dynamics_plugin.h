#ifndef TG1_OSP_DYNAMICS_H
#define TG1_OSP_DYNAMICS_H

// C++
#include <vector>
#include <iostream>     // std::cout, std::ios
#include <sstream>      // std::ostringstream

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>

//ROS
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
//#include <usv_msgs/UsvDrive.h>
//#include <usv_gazebo_plugins/UsvDrive.h>

#include <Eigen/Core>
//#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// gazebo_fmi
#include <functional>
#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// #include <gazebo_fmi/FMUCoSimulation.hh>


namespace gazebo
{

  class AutonoshipOSPPlugin : public ModelPlugin
  {
  public:
    AutonoshipOSPPlugin();
    virtual ~AutonoshipOSPPlugin();
    /*! Loads the model in gets dynamic parameters from SDF. */
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);


  protected:
    /*! Callback for Gazebo simulation engine */
    virtual void UpdateChild();
    virtual void FiniChild();
  private:
    /*! Presumably this would get called when there is a collision,
      but not implemented! */
    void OnContact(const std::string &name, const physics::Contact &contact);

    /*! ROS spin once */
    void spin();

    // bool LoadFMUs(gazebo::physics::ModelPtr _parent);

    /* update the inputs to OSP */
    void UpdateTargetCourse(const std_msgs::Float64 &target_course);
    void UpdateRPM(const std_msgs::Float64 &rpm);
    void UpdateLocalWaypoint(const geometry_msgs::Point &local_waypoint);
    void UpdateGlobalWaypoint(const geometry_msgs::Point &global_waypoint);

    /*! Convenience function for getting SDF parameters

     */
    double getSdfParamDouble(sdf::ElementPtr sdfPtr,const std::string &param_name,double default_val);

    /// Parameters
    std::string node_namespace_;
    std::string link_name_;

    ros::NodeHandle *rosnode_;

    ros::Publisher sensor_state_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher joint_state_pub_;
    ros::Publisher vel_x_pub_;
    ros::Publisher yaw_pub_;
    ros::Publisher twist_pub_;

    ros::Publisher init_time_pub_;
    ros::Publisher sim_time_pub_;

    ros::Subscriber target_course_sub_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber local_waypoint_sub_;
    ros::Subscriber global_waypoint_sub_;

    ros::ServiceClient OSP_client;

    //GazeboRosPtr gazebo_ros_;
    //physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;

    /*! Pointer to the Gazebo world, retrieved when the model is loaded */
    physics::WorldPtr world_;
    /*! Pointer to Gazebo parent model, retrieved when the model is loaded */
    physics::ModelPtr model_;
    /*! Pointer to model link in gazebo,
      optionally specified by the bodyName parameter,
      The states are taken from this link and forces applied to this link.*/
    physics::LinkPtr link_;


    /// \brief FMU
    // FMUShipDynamicsProperties m_fmu;

    // Simulation time of the last update
    common::Time prev_update_time_;
    ignition::math::Vector3d prev_lin_vel_;
    ignition::math::Vector3d prev_ang_vel_;
    ignition::math::Pose3d pose_;
    ignition::math::Vector3d euler_;
    ignition::math::Vector3d vel_linear_body_;
    ignition::math::Vector3d vel_angular_body_;
    ignition::math::Vector3d acceleration;
    ignition::math::Vector3d angular_velocity_;
    ignition::math::Vector3d angular_acceleration_;
    Eigen::VectorXd state_dot_;
    Eigen::VectorXd state_;
    Eigen::VectorXd amassVec_;
    Eigen::MatrixXd Cmat_;
    Eigen::VectorXd Cvec_;
    Eigen::MatrixXd Dmat_;
    Eigen::VectorXd Dvec_;

    Eigen::VectorXd TVec_;

    geometry_msgs::Twist twist_;

    bool is_ownship = false;

    // For Buoyancy calculation
    float buoy_factor_;

    // Values to set via Plugin Parameters
    /*! Plugin Parameter: Boat width [m] */
    double param_boat_width_;
    /*! Plugin Parameter: Boat length [m] */
    double param_boat_length_;
    /*! Plugin Parameter: Horizontal surface area [m^2] */
    double param_boat_area_ ;

    // tunable parameters of FMU, not integrated yet
    // double K_;  // K value of dynamics model
    // double T_;  // T value of dynamics model
    // double a_;  // a valud of dynamics model
    double r0_;  // initial angular velocity
    double phi0_;  // initial course
    double u0_;  // initial surge speed
    double v0_;  // initial sway speed
    double x0_;  // initial x coordinate
    double y0_;  // initial y coordinate

    /* inputs for OSP  */
    double target_course_ = 0.0;  // rudder input for rotation
    double rpm_ = 94.0;  // rudder input for acceleration
    double local_x_ = 0.0;
    double local_y_ = 0.0;
    double global_x_ = 0.0;
    double global_y_ = 0.0;
    double initialization_time;  // initialization time
    double initial_yaw;  // initial yaw

    double psi_;  // Angle
    double r_;  // AngularVelocity
    double previous_r_ = 0.0;
    double u_;  // Surge Velocity
    double previous_u_ = 0.0;
    double v_;  // Sway Velocity
    double previous_v_ = 0.0;
    double x_;  // x coordinate
    double y_;  // y coordinate

    double xyz_damping_;
    double yaw_damping_;
    double rp_damping_;
    /* Water height [m]*/
    double water_level_;
    /* Water density [kg/m^3] */
    double water_density_;
    /*! Added mass matrix, 6x6 */
    Eigen::MatrixXd Ma_;

    boost::thread *spinner_thread_;

    event::ConnectionPtr contact_event_;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

  };  // class AutonoshipOSPPlugin
} // namespace gazebo

#endif //TG1_OSP_DYNAMICS_H
