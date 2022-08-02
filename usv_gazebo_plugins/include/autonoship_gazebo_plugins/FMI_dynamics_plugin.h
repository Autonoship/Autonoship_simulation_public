/* 

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package, known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef FMI_DYNAMICS_H
#define FMI_DYNAMICS_H

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

#include <gazebo_fmi/FMUCoSimulation.hh>


namespace gazebo
{
  class FMUShipDynamicsProperties
  {
    /// \brief An identifier for the fmu
    public: std::string name;

    public: std::string fmuAbsolutePath;

    /// \brief Default tunable parameter names
    public: std::vector<std::string> m_tunableParameterDefaultNames;

    /// \brief Actual tunable parameter names, after parsing parameter_names
    public: std::vector<std::string> m_tunableParameterNames;

    /// \brief Default input variable names
    public: std::vector<std::string> m_inputVariablesDefaultNames;

    /// \brief Actual input variable names, after parsing variable_names
    public: std::vector<std::string> m_inputVariablesNames;

    /// \brief Default output variable names
    public: std::vector<std::string> m_outputVariablesDefaultNames;

    /// \brief Actual output variable names, after parsing variable_names
    public: std::vector<std::string> m_outputVariablesNames;

    public: FMUCoSimulation fmu;
    public: std::vector<fmi2_value_reference_t> inputVarReferences;
    public: std::vector<fmi2_value_reference_t> outputVarReferences;
    public: std::vector<fmi2_value_reference_t> tunableParReferences;
    public: std::vector<double> inputVarBuffers;
    public: std::vector<double> outputVarBuffers;
    public: std::vector<double> tunableParBuffers;

  };



  class AutonoshipFMUPlugin : public ModelPlugin
  {
  public:
    AutonoshipFMUPlugin();
    virtual ~AutonoshipFMUPlugin();
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

    bool LoadFMUs(gazebo::physics::ModelPtr _parent);

    /* the autonoship rudder command update */
    void UpdateU1(const std_msgs::Float64 &u1);
    void UpdateU2(const std_msgs::Float64 &u2);

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
    
    ros::Subscriber u1_sub_;
    ros::Subscriber u2_sub_;

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
    FMUShipDynamicsProperties m_fmu;
    
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

    // For Buoyancy calculation
    float buoy_factor_;
    
    // Values to set via Plugin Parameters
    /*! Plugin Parameter: Boat width [m] */
    double param_boat_width_;
    /*! Plugin Parameter: Boat length [m] */
    double param_boat_length_;
    /*! Plugin Parameter: Horizontal surface area [m^2] */
    double param_boat_area_ ;

    // tunable parameters of FMU 
    double K_;  // K value of dynamics model
    double T_;  // T value of dynamics model
    double a_;  // a valud of dynamics model
    double dphi0_;  // initial angular velocity
    double phi0_;  // initial course
    double u0_;  // initial linear speed
    double x0_;  // initial x coordinate
    double y0_;  // initial y coordinate

    /*  rudder input for autonoship  */
    double u1_;  // rudder input for rotation
    double u2_;  // rudder input for acceleration
    double initialization_time;  // initialization time 

    double psi_;  // Angle
    double r_;  // AngularVelocity
    double previous_r_;
    double u_;  // Velocity
    double previous_u_;
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
    
    /* Wave parameters */
    int param_wave_n_;
    std::vector<float> param_wave_amps_;
    std::vector<float> param_wave_periods_;
    std::vector< std::vector<float> > param_wave_directions_;
    /* Old - for single wave
    double param_wave_amp_;
    math::Vector2d param_wave_dir_;
    double param_wave_period_;
    */

    /*! Wind velocity in Gazebo coordinates [m/s] */
    ignition::math::Vector3d param_wind_velocity_vector_;

    /*! Wind force coefficients */
    ignition::math::Vector3d param_wind_coeff_vector_;
    
    boost::thread *spinner_thread_;
    
    event::ConnectionPtr contact_event_;
    
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

  };  // class AutonoshipFMUPlugin
} // namespace gazebo

#endif //FMI_DYNAMICS_H
