// Copyright (c) 2018 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschresearch/fmi_adapter.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>

#include <exception>
#include <map>
#include <string>

#include <ros/ros.h>
#include <fmi_adapter/FMIAdapter.h>
#include <std_msgs/Float64.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "fmi_adapter_examples_integrated_node");
  ros::NodeHandle n("~");

  std::string pendulumFmuPath;
  if (!n.getParam("pendulum_fmu_path", pendulumFmuPath)) {
    ROS_ERROR("Parameter 'pendulum_fmu_path' not specified!");
    throw std::runtime_error("Parameter 'pendulum_fmu_path' not specified!");
  }

  std::string delayFmuPath;
  if (!n.getParam("delay_fmu_path", delayFmuPath)) {
    ROS_ERROR("Parameter 'delay_fmu_path' not specified!");
    throw std::runtime_error("Parameter 'delay_fmu_path' not specified!");
  }

  ros::Duration stepSize(0.02);
  fmi_adapter::FMIAdapter pendulumAdapter(pendulumFmuPath, stepSize);
  pendulumAdapter.setInitialValue("d", 0.001);
  pendulumAdapter.setInitialValue("l", 2.0);
  fmi_adapter::FMIAdapter delayAdapter(delayFmuPath, stepSize);
  delayAdapter.setInitialValue("d", 5.5);

  pendulumAdapter.setInitialValue("a", 1.3);

  ros::Time now = ros::Time::now();
  pendulumAdapter.exitInitializationMode(now);
  delayAdapter.exitInitializationMode(now);

  ros::Publisher anglePub = n.advertise<std_msgs::Float64>("angle", 1000);
  ros::Publisher delayedAnglePub = n.advertise<std_msgs::Float64>("delayed_angle", 1000);

  ros::Timer timer = n.createTimer(stepSize, [&](const ros::TimerEvent& event) {
    pendulumAdapter.doStepsUntil(event.current_expected);
    double angle = pendulumAdapter.getOutputValue("a");
    delayAdapter.setInputValue("x", event.current_expected, angle);
    delayAdapter.doStepsUntil(event.current_expected);
    double delayedAngle = delayAdapter.getOutputValue("y");
    std_msgs::Float64 angleMsg;
    angleMsg.data = angle;
    anglePub.publish(angleMsg);
    std_msgs::Float64 delayedAngleMsg;
    delayedAngleMsg.data = delayedAngle;
    delayedAnglePub.publish(delayedAngleMsg);
  });

  ros::spin();
}
