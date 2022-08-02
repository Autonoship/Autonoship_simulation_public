#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>

#include <autonoship_gazebo_plugins/OSP_dynamics_plugin.h>
#include <experimental/filesystem>

#include <osp_ros_demo/updateState.h>

#include <gazebo/sensors/sensors.hh>

#define GRAVITY 9.815

using namespace gazebo;

AutonoshipOSPPlugin::AutonoshipOSPPlugin()
{
}

AutonoshipOSPPlugin::~AutonoshipOSPPlugin()
{
  rosnode_->shutdown();
  spinner_thread_->join();
  delete rosnode_;
  delete spinner_thread_;
}

void AutonoshipOSPPlugin::FiniChild()
{
}


double AutonoshipOSPPlugin::getSdfParamDouble(sdf::ElementPtr sdfPtr, const std::string &param_name, double default_val)
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

void AutonoshipOSPPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ROS_INFO("Loading autonoship_dynamics_plugin");
  model_ = _parent;
  world_ = model_->GetWorld();

  // auto cam = sensors::SensorManager::Instance()->GetSensor("gzclient_camera");
  // cam.camera

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
  if (model_->GetName() == "ownship")
  {
    is_ownship = true;
  }
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
    link_ = model_->GetLink(link_name_);
    ROS_INFO_STREAM("Found SDF parameter bodyName as <"<<link_name_<<">");
  }
  if (!link_)
  {
    ROS_FATAL("OSP_dynamics_plugin error: bodyName: %s does not exist\n", link_name_.c_str());
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

  target_speed_ = 0.0;
  target_course_ = 0.0;

  std::string physicsEngineName;
  physicsEngineName = gazebo::physics::get_world()->Physics()->GetType();

  if (physicsEngineName == "bullet")
  {
    gzerr << "FMIShipDynamicsPlugin: the bullet physics engine is not supported, since using Bullet in Gazebo there is no way to excert an external force on a link." << std::endl;
    gzerr << "See FMISingleBodyFluidDynamicsPlugin: see https://bitbucket.org/osrf/gazebo/issues/1476/implement-addxxxforce-for-bullet" << std::endl;
    return;
  }

  // Initialize the ROS node
  int argc = 0;
  char** argv = NULL;

  ros::init(argc, argv, "OSP_dynamics_gazebo",
  ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  OSP_client = rosnode_->serviceClient<osp_ros_demo::updateState>("update_OSP");
  ROS_INFO_STREAM("Set OSP Client");

  /*
  init_time_pub_ = rosnode_->advertise<std_msgs::Float64>("init_time", 1, true);
  sim_time_pub_ = rosnode_->advertise<std_msgs::Float64>("sim_time", 1, true);
  */

  double simulatedTimeInSeconds  = _parent->GetWorld()->SimTime().Double();
  initialization_time = simulatedTimeInSeconds;

  pose_ = link_->WorldPose();
  euler_ = pose_.Rot().Euler();
  initial_yaw = euler_.Z();

  /*
  std_msgs::Float64 init_time;
  init_time.data = initialization_time;
  init_time_pub_.publish(init_time);

  std_msgs::Float64 sim_time;
  sim_time.data = simulatedTimeInSeconds;
  sim_time_pub_.publish(sim_time);
  */

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

  /* listen to the target inputs */
  target_course_sub_ = rosnode_->subscribe("local_target_course", 1, &AutonoshipOSPPlugin::UpdateTargetCourse, this );
  target_speed_sub_ = rosnode_->subscribe("setpoint", 1, &AutonoshipOSPPlugin::UpdateTargetSpeed, this );

  buoy_factor_ = param_boat_area_*GRAVITY*water_density_;

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->spinner_thread_ = new boost::thread( boost::bind( &AutonoshipOSPPlugin::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
  boost::bind(&AutonoshipOSPPlugin::UpdateChild, this));

}


void AutonoshipOSPPlugin::UpdateChild()
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

  /**/
  std_msgs::Float64 vel_x_;
  vel_x_.data = vel_linear_body_.X();
  vel_x_pub_.publish(vel_x_);


  std_msgs::Float64 yaw_;
  yaw_.data = euler_.Z();
  yaw_pub_.publish(yaw_);

  double simulatedTimeInSeconds  = time_now.Double();
  // std_msgs::Float64 sim_time;
  // sim_time.data = simulatedTimeInSeconds;
  // sim_time_pub_.publish(sim_time);

  // gzmsg << "simulatedTimeInSeconds: " << simulatedTimeInSeconds << std::endl;
  auto world = this->world_;
  double stepSizeInSeconds = world->Physics()->GetMaxStepSize();
  // gzmsg << "stepSizeInSeconds: " << stepSizeInSeconds << std::endl;


  /*
  if (simulate_steps < 10)
  {
    simulate_steps += 1;
    delta_time += dt;
  }
  else
  {
    // Set inputs
    osp_ros_demo::updateState srv;
    srv.request.time.data = simulatedTimeInSeconds - initialization_time;
    srv.request.target_speed.data = target_speed_;
    srv.request.target_course.data = target_course_ - euler_.Z();

    OSP_client.call(srv);

    double angular_accel = (- srv.response.phi_dot.data - previous_r_) / delta_time;  // the srv.response.phi_dot.data is the time deritive of the course, which is left-handed
    previous_r_ = - srv.response.phi_dot.data;
    double linear_accel = (srv.response.v.data - previous_u_) / delta_time;
    previous_u_ = srv.response.v.data;

    simulate_steps = 0;
    delta_time = 0.0;

    gzmsg << "Linear acceleration: " << linear_accel << std::endl;
    gzmsg << "Angular acceleration: " << angular_accel << std::endl;

    link_->AddRelativeForce(ignition::math::Vector3d(linear_accel*143620, u_*r_*143620, 0.0));
    link_->AddRelativeTorque(ignition::math::Vector3d(0.0, 0.0, angular_accel*1357268841));
  }
  */

  // Set inputs
  osp_ros_demo::updateState srv;
  srv.request.time.data = simulatedTimeInSeconds - initialization_time;
  srv.request.target_speed.data = target_speed_;
  srv.request.target_course.data = target_course_ - (euler_.Z() - initial_yaw);

  OSP_client.call(srv);

  r_ = -srv.response.phi_dot.data;  // the srv.response.phi_dot.data is the time deritive of the course, which is left-handed
  double angular_accel = (r_ - previous_r_)/dt;
  previous_r_ = r_;
  u_ = srv.response.v.data;
  double linear_accel = (u_ - previous_u_)/dt;
  double vel_x_bias = previous_u_ - vel_linear_body_.X();
  previous_u_ = u_;

  /*
  gzmsg << "Time: " << time_now << std::endl;
  gzmsg << model_->GetName() << std::endl;
  gzmsg << "target v: " << target_speed_ << std::endl;
  gzmsg << "OSP v: " << srv.response.v.data << std::endl;
  gzmsg << "current v: " << vel_linear_body_.X() << std::endl << std::endl;
  */

  /*  
  gzmsg << "simulate to: " << simulatedTimeInSeconds - initialization_time << std::endl;
  gzmsg << "r: " << srv.response.phi_dot.data << std::endl;
  gzmsg << "phi: " << srv.response.phi.data << std::endl;
  gzmsg << "x: " << srv.response.x.data << std::endl;
  gzmsg << "y: " << srv.response.y.data << std::endl;
  gzmsg << "v: " << srv.response.v.data << std::endl;
  */
  
  // gzmsg << "relative vel_y: " << vel_linear_body_.Y() << std::endl;
  
  
  /*
  gzmsg << "simulate to: " << simulatedTimeInSeconds - initialization_time << std::endl;
  // gzmsg << "delta_r: " << srv.response.phi_dot.data << std::endl;
  gzmsg << "delta_phi: " << srv.response.phi.data + twist_.angular.z << std::endl;
  gzmsg << "delta_x: " << srv.response.x.data - twist_.linear.x << std::endl;
  gzmsg << "delta_y: " << srv.response.y.data + twist_.linear.y << std::endl;
  gzmsg << "delta_v: " << srv.response.v.data - vel_linear_body_.X() << std::endl;
  */
  
  double relative_force_x = (vel_x_bias / dt + linear_accel) * 143620;
  double relative_force_y = (-vel_linear_body_.Y() / dt + u_*r_) * 143620;

  link_->AddRelativeForce(ignition::math::Vector3d(relative_force_x, relative_force_y, 0.0));
  link_->AddRelativeTorque(ignition::math::Vector3d(0.0, 0.0, angular_accel*1357268841));

  // Distribute upward buoyancy force
  float buoy_force;
  double vel_z = vel_linear_body_.Z();
  buoy_force = (water_level_ - twist_.linear.z - 0.1*vel_z)*buoy_factor_;
  // buoy_force = 143620 * GRAVITY;
  /*
  gzmsg << "vel_z: " << vel_z << std::endl;
  gzmsg << "twist_.linear.z: " << twist_.linear.z << std::endl;
  gzmsg << "buoy_force: " << buoy_force << std::endl;
  */

  link_->AddRelativeForce(ignition::math::Vector3d(0.0, 0.0, buoy_force));
  // link_->AddRelativeTorque(ignition::math::Vector3d(0.0, 0.0, 0.0));
  
  
  
  /*
  // update camera pose (failed to update global gzclient_camera)
  if (is_ownship)
  {
    // gzmsg << "Moving gzclient_camera ..." << std::endl;
    auto cam = sensors::SensorManager::Instance()->GetSensor("gzclient_camera");
    if (cam == nullptr)
    {
      gzmsg << "Failed to find gzclient_camera." << std::endl;
      auto all_sensors = sensors::SensorManager::Instance()->GetSensors();
      for (auto it = begin(all_sensors); it != end(all_sensors); ++it)
      {
        gzmsg << (*it)->Name() << std::endl;
      }
    }
    else
    {
      ignition::math::Pose3d pose = cam->Pose();
      ignition::math::Vector3d Pos = pose.Pos();
      Pos.Set(twist_.linear.x, twist_.linear.y, pose.Z());
      pose.Set(Pos, pose.Rot());
      cam->SetPose(pose);
    }
  }
  */
  
}

void AutonoshipOSPPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

void AutonoshipOSPPlugin::UpdateTargetCourse(const std_msgs::Float64 &course)
{
    target_course_ = course.data;
    ROS_DEBUG_STREAM("Receiving target_course: " << target_course_);
}

void AutonoshipOSPPlugin::UpdateTargetSpeed(const std_msgs::Float64 &speed)
{
    target_speed_ = speed.data;
    ROS_DEBUG_STREAM("Receiving target_speed: " << target_speed_);
}

GZ_REGISTER_MODEL_PLUGIN(AutonoshipOSPPlugin);

