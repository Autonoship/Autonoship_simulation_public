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
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <gazebo_fmi/FMUCoSimulation.hh>

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>

#include <autonoship_gazebo_plugins/FMI_dynamics_plugin.h>

#include <gazebo_fmi/SDFConfigurationParsing.hh>
#include <experimental/filesystem>

#define GRAVITY 9.815

using namespace gazebo;

namespace FMIShipDynamicsPluginNS
{
  enum InputIndex
  {
    delta = 0,
    target_u,
    TotalInputs,
  };

  enum OutputIndex
  {
    r = 0,
    psi,
    x,
    y,
    u,
    TotalOutputs,
  };

  enum TunableParameterIndex
  {
    K = 0,
    T,
    a,
    dphi0,
    phi0,
    u0,
    x0,
    y0,
    TotalParameters,
  };
}


AutonoshipFMUPlugin::AutonoshipFMUPlugin()
{
}

AutonoshipFMUPlugin::~AutonoshipFMUPlugin()
{
  rosnode_->shutdown();
  spinner_thread_->join();
  delete rosnode_;
  delete spinner_thread_;
}

void AutonoshipFMUPlugin::FiniChild()
{
}
    

double AutonoshipFMUPlugin::getSdfParamDouble(sdf::ElementPtr sdfPtr, const std::string &param_name, double default_val)
{
  double val = default_val;
  if (sdfPtr->HasElement(param_name) && sdfPtr->GetElement(param_name)->GetValue()) 
  {
    val = sdfPtr->GetElement(param_name)->Get<double>();
    ROS_INFO_STREAM("Parameter found - setting <" << param_name << "> to <" << val << ">.");

  }
  else{
    ROS_INFO_STREAM("Parameter <" << param_name << "> not found: Using default value of <" << val << ">.");
  }
  return val;
}
				    
void AutonoshipFMUPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ROS_INFO("Loading autonoship_dynamics_plugin");
  model_ = _parent;
  world_ = model_->GetWorld();


  // Retrieve model paramters from SDF
  // Set default values
  node_namespace_ = "";

  water_level_ = 0.5;
  water_density_ = 997.7735;

  param_boat_width_ = 1.0;
  param_boat_length_ = 1.35;
  param_boat_area_ = param_boat_width_ * param_boat_length_;

  K_ = 0.1555; 
  T_ = 73.77; 
  a_ = 0.034296296296; 
  dphi0_ = 0.0; 
  phi0_ = 0.0; 
  u0_ = 0.0; 
  x0_ = 0.0; 
  y0_ = 0.0; 

  
  //  Enumerating model
  ROS_INFO_STREAM("Enumerating Model...");
  ROS_INFO_STREAM("Model name = "<< model_->GetName());
  physics::Link_V links = model_->GetLinks();
  for (int ii=0; ii<links.size(); ii++){
    ROS_INFO_STREAM("Link: "<<links[ii]->GetName());
  }

  // Get parameters from SDF
  if (_sdf->HasElement("robotNamespace")) 
  {
    node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (!_sdf->HasElement("bodyName") || !_sdf->GetElement("bodyName")->GetValue())
  {                       
    link_ = model_->GetLink();
    link_name_ = link_->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  } 
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    //link_name_ = "thrust_link";
    link_ = model_->GetLink(link_name_);

    ROS_INFO_STREAM("Found SDF parameter bodyName as <"<<link_name_<<">");
  }
  if (!link_)
  {
    ROS_FATAL("autonoship_dynamics_plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }
  else
  {
    ROS_INFO_STREAM("Autonoship Model Link Name = " << link_name_);
  }

  water_level_ = getSdfParamDouble(_sdf,"waterLevel",water_level_);
  water_density_ = getSdfParamDouble(_sdf,"waterDensity",water_density_);

  param_boat_area_ = getSdfParamDouble(_sdf,"boatArea",param_boat_area_);
  param_boat_width_ = getSdfParamDouble(_sdf,"boatWidth",param_boat_width_);
  param_boat_length_ = getSdfParamDouble(_sdf,"boatLength",param_boat_length_);

  if (_sdf->HasElement("ship_dynamics")) {
    if (_sdf->GetElement("ship_dynamics")->HasElement("tunable_parameter")) {
      sdf::ElementPtr tunable_elem = _sdf->GetElement("ship_dynamics")->GetElement("tunable_parameter");
      K_ = getSdfParamDouble(tunable_elem,"K",K_);
      T_ = getSdfParamDouble(tunable_elem,"T",T_);
      a_ = getSdfParamDouble(tunable_elem,"a",a_);
      dphi0_ = getSdfParamDouble(tunable_elem,"dphi0",dphi0_);
      phi0_ = getSdfParamDouble(tunable_elem,"phi0",phi0_);
      u0_ = getSdfParamDouble(tunable_elem,"u0",u0_);
      x0_ = getSdfParamDouble(tunable_elem,"x0",x0_);
      y0_ = getSdfParamDouble(tunable_elem,"y0",y0_);
      ROS_INFO_STREAM("Set tunable parameters");
    }
  }


  u1_ = 0.0;
  u2_ = 0.0;


  // Configure default FMU variable names
  m_fmu.m_inputVariablesDefaultNames.resize(FMIShipDynamicsPluginNS::TotalInputs);
  m_fmu.m_inputVariablesDefaultNames[FMIShipDynamicsPluginNS::delta] = "delta";
  m_fmu.m_inputVariablesDefaultNames[FMIShipDynamicsPluginNS::target_u] = "target_u";

  m_fmu.m_tunableParameterDefaultNames.resize(FMIShipDynamicsPluginNS::TotalParameters);
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::K] = "K";
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::T] = "T";
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::a] = "a";
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::dphi0] = "dphi0";
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::phi0] = "phi0";
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::u0] = "u0";
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::x0] = "x0";
  m_fmu.m_tunableParameterDefaultNames[FMIShipDynamicsPluginNS::y0] = "y0";

  m_fmu.m_outputVariablesDefaultNames.resize(FMIShipDynamicsPluginNS::TotalOutputs);
  m_fmu.m_outputVariablesDefaultNames[FMIShipDynamicsPluginNS::r] = "r";
  m_fmu.m_outputVariablesDefaultNames[FMIShipDynamicsPluginNS::psi] = "psi";
  m_fmu.m_outputVariablesDefaultNames[FMIShipDynamicsPluginNS::x] = "x";
  m_fmu.m_outputVariablesDefaultNames[FMIShipDynamicsPluginNS::y] = "y";
  m_fmu.m_outputVariablesDefaultNames[FMIShipDynamicsPluginNS::u] = "u";

  ROS_INFO_STREAM("Set default FMU variable names");



  std::string physicsEngineName;
  physicsEngineName = gazebo::physics::get_world()->Physics()->GetType();

  if (physicsEngineName == "bullet")
  {
    gzerr << "FMIShipDynamicsPlugin: the bullet physics engine is not supported, since using Bullet in Gazebo there is no way to excert an external force on a link." << std::endl;
    gzerr << "See FMISingleBodyFluidDynamicsPlugin: see https://bitbucket.org/osrf/gazebo/issues/1476/implement-addxxxforce-for-bullet" << std::endl;
    return;
  }


  // Parse parameters
  sdf::ElementPtr elem = _sdf->GetElement("ship_dynamics");
  if (elem->HasElement("name"))
    m_fmu.name = elem->Get<std::string>("name");
  m_fmu.fmuAbsolutePath =  gazebo::common::SystemPaths::Instance()->FindFile(elem->Get<std::string>("fmu"));
  if (!std::experimental::filesystem::exists(m_fmu.fmuAbsolutePath))
  {
    gzerr << "Could not find FMU named " << elem->Get<std::string>("fmu")
          << " in the GAZEBO_RESOURCE_PATH directories" << std::endl;
    return;
  }

  // Process variable_names element
  bool variableNamesOk = gazebo::parseVariableNamesSDFElement(elem, m_fmu.m_inputVariablesDefaultNames, m_fmu.m_inputVariablesNames, m_fmu.m_outputVariablesDefaultNames, m_fmu.m_outputVariablesNames, m_fmu.m_tunableParameterDefaultNames, m_fmu.m_tunableParameterNames);

  if (!variableNamesOk)
  {
    gzerr << "FMIShipDynamicsPlugin: failure in parsing variable_names tag" << std::endl;
    return;
  }
  
  // Initialize the ROS node
  int argc = 0;
  char** argv = NULL;
  
  ros::init(argc, argv, "autonoship_dynamics_gazebo", 
	    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  init_time_pub_ = rosnode_->advertise<std_msgs::Float64>("init_time", 1, true);
  sim_time_pub_ = rosnode_->advertise<std_msgs::Float64>("sim_time", 1, true);
  
// load FMU file
  double simulatedTimeInSeconds  = _parent->GetWorld()->SimTime().Double();
  initialization_time = simulatedTimeInSeconds;
  
  std_msgs::Float64 init_time;
  init_time.data = initialization_time;
  init_time_pub_.publish(init_time);
  
  std_msgs::Float64 sim_time;
  sim_time.data = simulatedTimeInSeconds;
  sim_time_pub_.publish(sim_time);
  
  
  std::string instanceName = link_->GetScopedName()+"_fmuShipDynamics";
  // simulatedTimeInSeconds = 0.0;
  bool ok = m_fmu.fmu.load(m_fmu.fmuAbsolutePath, instanceName, simulatedTimeInSeconds - initialization_time);

  gzmsg << "Setting the initial time: " << simulatedTimeInSeconds << " to 0" << std::endl;

  ROS_INFO_STREAM("Loaded FMU file");



  if (!ok) {
    gzerr << "FMIShipDynamicsPlugin: failure in loading FMU file" << std::endl;
    return;
  }

  // Get references for input variables
  ok = m_fmu.fmu.getInputVariableRefs(m_fmu.m_inputVariablesNames, m_fmu.inputVarReferences);
  if (!ok) {
    gzerr << "FMIShipDynamicsPlugin: failure in getting input variables" << std::endl;
    return;
  }
  m_fmu.inputVarBuffers.resize(m_fmu.inputVarReferences.size());

  // Get references for tunable parameters
  ok = m_fmu.fmu.getTunableParameterRefs(m_fmu.m_tunableParameterNames, m_fmu.tunableParReferences);
  if (!ok) {
    gzerr << "FMIShipDynamicsPlugin: failure in getting tunable parameters" << std::endl;
    return;
  }
  m_fmu.tunableParBuffers.resize(m_fmu.tunableParReferences.size());

  // Get references for output variables
  ok = m_fmu.fmu.getOutputVariableRefs(m_fmu.m_outputVariablesNames, m_fmu.outputVarReferences);
  if (!ok) {
    gzerr << "FMIShipDynamicsPlugin: failure in getting output variables" << std::endl;
    return;
  }
  m_fmu.outputVarBuffers.resize(m_fmu.outputVarBuffers.size());

  // test //
  // get initial pose of the ship //
  pose_ = link_->WorldPose();
  ROS_INFO_STREAM(model_->GetName() << " initial pose: " << pose_);


  // Get inertia and mass of vessel
  ignition::math::Vector3d inertia = link_->GetInertial()->PrincipalMoments();
  double mass = link_->GetInertial()->Mass();

  // Report some of the pertinent parameters for verification
  ROS_INFO("Autonoship Dynamics Parameters: From URDF XACRO model definition");
  ROS_INFO_STREAM("Vessel Mass (rigid-body): " << mass);
  ROS_INFO_STREAM("Vessel Inertia Vector (rigid-body): X:" << inertia[0] <<
		  " Y:"<<inertia[1] <<
		  " Z:"<<inertia[2]);
  ROS_INFO("Autonoship Dynamics Parameters: Plugin Parameters");

  //initialize time and odometry position
  prev_update_time_ = this->world_->SimTime();
  gzmsg << "prev_update_time_: " << prev_update_time_ << std::endl;
  
  vel_x_pub_ = rosnode_->advertise<std_msgs::Float64>("vel_x", 1, true);
  yaw_pub_ = rosnode_->advertise<std_msgs::Float64>("yaw", 1, true);
  twist_pub_ = rosnode_->advertise<geometry_msgs::Twist>("twist", 1, true);

  /* listen to the rudder command */
  u1_sub_ = rosnode_->subscribe("u1", 1, &AutonoshipFMUPlugin::UpdateU1, this );
  u2_sub_ = rosnode_->subscribe("u2", 1, &AutonoshipFMUPlugin::UpdateU2, this );

  buoy_factor_ = param_boat_area_*GRAVITY*water_density_;

  // Set tunable parameters
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::K] = K_;
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::T] = T_;
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::a] = a_;
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::dphi0] = dphi0_;
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::phi0] = phi0_;
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::u0] = u0_;
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::x0] = x0_;
  m_fmu.tunableParBuffers[FMIShipDynamicsPluginNS::y0] = y0_;
  ok = m_fmu.fmu.setTunableParameters(m_fmu.tunableParReferences, m_fmu.tunableParBuffers);
  if (ok) {
    gzmsg << "tunable parameters set" << std::endl;
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ = new boost::thread( boost::bind( &AutonoshipFMUPlugin::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&AutonoshipFMUPlugin::UpdateChild, this));


  
}


void AutonoshipFMUPlugin::UpdateChild()
{
  common::Time time_now = this->world_->SimTime();
  //common::Time step_time = time_now - prev_update_time_;
  double dt = (time_now - prev_update_time_).Double();
  prev_update_time_ = time_now;
  
  
  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  pose_ = link_->WorldPose();
  euler_ = pose_.Rot().Euler();
 
  twist_.linear.x = pose_.X();
  twist_.linear.y = pose_.Y();
  twist_.linear.z = pose_.Z();
  twist_.angular.x = euler_.X();
  twist_.angular.y = euler_.Y();
  twist_.angular.z = euler_.Z();
  twist_pub_.publish(twist_);

  // Get body-centered linear and angular rates
  vel_linear_body_ = link_->RelativeLinearVel();  
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel linear: " << vel_linear_body_);
  vel_angular_body_ = link_->RelativeAngularVel();
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel angular: " << vel_angular_body_);
  // Estimate the linear and angular accelerations.
  // Note the the GetRelativeLinearAccel() and AngularAccel() functions
  // appear to be unreliable

  // ROS_INFO_STREAM(model_->GetName() << " vel_linear_body_: "<< vel_linear_body_
  //                                   << " at time: " << time_now);

  std_msgs::Float64 vel_x_;
  vel_x_.data = vel_linear_body_.X();
  vel_x_pub_.publish(vel_x_);

  std_msgs::Float64 yaw_;
  yaw_.data = euler_.Z();
  yaw_pub_.publish(yaw_);

  double simulatedTimeInSeconds  = time_now.Double();
  std_msgs::Float64 sim_time;
  sim_time.data = simulatedTimeInSeconds;
  sim_time_pub_.publish(sim_time);
  gzmsg << "simulatedTimeInSeconds: " << simulatedTimeInSeconds << std::endl;
  auto world = this->world_;
  double stepSizeInSeconds = world->Physics()->GetMaxStepSize();
  gzmsg << "stepSizeInSeconds: " << stepSizeInSeconds << std::endl;


  // Set inputs
  m_fmu.inputVarBuffers[FMIShipDynamicsPluginNS::delta] = u1_;
  m_fmu.inputVarBuffers[FMIShipDynamicsPluginNS::target_u] = u2_;

  bool ok = m_fmu.fmu.setInputVariables(m_fmu.inputVarReferences, m_fmu.inputVarBuffers);
  if (!ok) {
    gzmsg << "Fail to set input variables" << std::endl;
  }


  // Run fmu simulation
  ok = ok && m_fmu.fmu.doStep(simulatedTimeInSeconds - stepSizeInSeconds - initialization_time, stepSizeInSeconds);

  // Get ouput
  ok = ok && m_fmu.fmu.getOutputVariables(m_fmu.outputVarReferences, m_fmu.outputVarBuffers);

  if (!ok)
  {
    gzerr << "gazebo_fmi: Failure in simulating single body fluid dynamics forces of link " << link_->GetScopedName() << std::endl;
  }

  // This order should be coherent with the order defined in Load
  psi_ = m_fmu.outputVarBuffers[FMIShipDynamicsPluginNS::psi];
  r_ = m_fmu.outputVarBuffers[FMIShipDynamicsPluginNS::r];
  u_ = m_fmu.outputVarBuffers[FMIShipDynamicsPluginNS::u];
  x_ = m_fmu.outputVarBuffers[FMIShipDynamicsPluginNS::x];
  y_ = m_fmu.outputVarBuffers[FMIShipDynamicsPluginNS::y];

  double angular_accel = (r_ - previous_r_)/dt;
  previous_r_ = r_;
  double linear_accel = (u_ - previous_u_)/dt;
  previous_u_ = u_;

// Distribute upward buoyancy force
  float buoy_force;
  double vel_z = vel_linear_body_.Z();
  buoy_force = (water_level_ - twist_.linear.z - 1.0*vel_z)*buoy_factor_;

  link_->AddRelativeForce(ignition::math::Vector3d(linear_accel*143620, u_*r_*143620, buoy_force));
  link_->AddRelativeTorque(ignition::math::Vector3d(0.0, 0.0, angular_accel*1357268841));
  
}

void AutonoshipFMUPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

void AutonoshipFMUPlugin::UpdateU1(const std_msgs::Float64 &u1)
{
    u1_ = u1.data;
    ROS_DEBUG_STREAM("Receiving u1: " << u1_);
}

void AutonoshipFMUPlugin::UpdateU2(const std_msgs::Float64 &u2)
{
    u2_ = u2.data;
    ROS_DEBUG_STREAM("Receiving u2: " << u2_);
}

GZ_REGISTER_MODEL_PLUGIN(AutonoshipFMUPlugin);

