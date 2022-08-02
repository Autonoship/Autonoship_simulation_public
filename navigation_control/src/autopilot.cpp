#include <cassert>

#include <exception>
#include <map>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>

#include <fmi_adapter/FMIAdapter.h>


double delta = 0;
double u1 = 0;
double init_time = 0;
double sim_time = 0;

ros::Time simTime(0.0);

void deltaCallback(const std_msgs::Float64 &msg)
{
  delta = msg.data;
}

void initTimeCallback(const std_msgs::Float64 &msg)
{
  init_time = msg.data;
}

void simTimeCallback(const std_msgs::Float64 &msg)
{
  // sim_time = msg.data;
  simTime = ros::Time(msg.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot");
  ros::NodeHandle n;
  
  ros::Subscriber init_time_sub = n.subscribe("init_time", 1, initTimeCallback);
  ros::Subscriber sim_time_sub = n.subscribe("sim_time", 1, simTimeCallback);
  
  // ros::Publisher u1_pub = n.advertise<std_msgs::Float64>("u1", 1);
  // ros::Subscriber delta_sub = n.subscribe("delta", 1, deltaCallback);

  std::string autopilotType;
  n.getParam("autopilot/autopilot_type", autopilotType);
  
  std::string fmuPath;
  if (!n.getParam("autopilot/fmu_path", fmuPath)) {
    ROS_ERROR("Parameter 'fmu_path' not specified!");
    // throw std::runtime_error("Parameter 'fmu_path' not specified!");
  }
  
  double stepSizeAsDouble = 0.001;
  n.getParam("autopilot/step_size", stepSizeAsDouble);
  ros::Duration stepSize(stepSizeAsDouble);

  fmi_adapter::FMIAdapter adapter(fmuPath, stepSize);
  for (const std::string & name : adapter.getParameterNames()) {
    ROS_DEBUG("FMU has parameter '%s'", name.c_str());
  }
  adapter.initializeFromROSParameters(n);

  std::string strPID("PID");
  if (!strPID.compare(autopilotType)) { // if the autopilot type is PID, set tunable parameters
    ROS_INFO("autopilotType is PID.");
    double P = 0.02;
    double I = 0.0;
    double D = 0;
    n.getParam("autopilot/P", P);
    n.getParam("autopilot/I", I);
    n.getParam("autopilot/D", D);
    adapter.setInitialValue("P.Value", P);
    adapter.setInitialValue("I.Value", I);
    adapter.setInitialValue("D.Value", D);
  }
  
  std::map<std::string, ros::Subscriber> subscribers;
  for (const std::string& name : adapter.getInputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    ros::Subscriber subscriber =
        n.subscribe<std_msgs::Float64>(rosifiedName, 1000, [&adapter, name](const std_msgs::Float64::ConstPtr& msg) {
          std::string myName = name;
          adapter.setInputValue(myName, simTime, msg->data);
        });
    subscribers[name] = subscriber;
  }


  std::map<std::string, ros::Publisher> publishers;
  for (const std::string& name : adapter.getOutputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    publishers[name] = n.advertise<std_msgs::Float64>(rosifiedName, 1000);
  }

  adapter.exitInitializationMode(simTime);

  ros::Rate loop_rate(1000);  // 1000Hz
  while (ros::ok())
  {
    if (adapter.getSimulationTime() < simTime) {
      adapter.doStepsUntil(simTime);
    }
    for (const std::string& name : adapter.getOutputVariableNames()) {
      std_msgs::Float64 msg;
      msg.data = adapter.getOutputValue(name);
      publishers[name].publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  // ros::spin();

  return 0;
}

