/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */
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
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "fmi_adapter/FMIAdapter.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "fmi_adapter_node");
  ros::NodeHandle n("~");

  // Check for a valid FMU
  std::string fmuPath;
  if (!n.getParam("fmu_path", fmuPath)) {
    ROS_ERROR("Parameter 'fmu_path' not specified!");
    throw std::runtime_error("Parameter 'fmu_path' not specified!");
  }

  // Get the Step-Size
  double stepSizeAsDouble = 0.0;
  n.getParam("step_size", stepSizeAsDouble);
  ros::Duration stepSize(stepSizeAsDouble);


  // Create the Adapter
  ROS_DEBUG("Creating Adapter");
  fmi_adapter::FMIAdapter adapter(fmuPath, stepSize);
  ROS_DEBUG("Adapter created");

  for (auto const& element : adapter.getParameterNamesAndBaseTypes()) {
    ROS_DEBUG("FMU has parameter '%s'", element.first.c_str());
  }

  // Init the Adapter
  ROS_DEBUG("Init Start");
  adapter.initializeFromROSParameters(n);


  ROS_DEBUG("Creating subscribers...");
  // Create a map of all subscribers, accessible by their names
  std::map<std::string, ros::Subscriber> subscribers;

  // Iterate over all FMU Input Variables and subscribe the adapter inputs accordingly
  for (auto const& element : adapter.getInputVariableNamesAndBaseTypes()) {

    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(element.first);

    ros::Subscriber subscriber;

    switch(element.second) {
    case fmi2_base_type_real:
      subscriber = n.subscribe<std_msgs::Float64>(
        rosifiedName, 1000,
        [&adapter, element](const std_msgs::Float64::ConstPtr& msg) {
          std::string myName = element.first;
          adapter.setInputValue(myName, ros::Time::now(), msg->data);
        }
      );
      break;

    case fmi2_base_type_bool:
      subscriber = n.subscribe<std_msgs::Bool>(
        rosifiedName, 1000,
        [&adapter, element](const std_msgs::Bool::ConstPtr& msg) {
          std::string myName = element.first;
          adapter.setInputValue(myName, ros::Time::now(), msg->data);
        }
      );
      break;

    case fmi2_base_type_int:
      break;
    case fmi2_base_type_str:
      break;
    case fmi2_base_type_enum:
      break;
    }


    if(subscriber)
      subscribers[element.first] = subscriber;

  }

  ROS_DEBUG("Creating publishers...");
  // Create a map of all publishers, accessible by their names
  std::map<std::string, ros::Publisher> publishers;

  // Iterate over all FMU Output Variables and publish the adapter outputs accordingly
  for (auto const& element : adapter.getOutputVariableNamesAndBaseTypes()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(element.first);

    switch(element.second) {
    case fmi2_base_type_real:
      publishers[element.first] = n.advertise<std_msgs::Float64>(rosifiedName, 1000);
      break;
    case fmi2_base_type_bool:
      publishers[element.first] = n.advertise<std_msgs::Bool>(rosifiedName, 1000);
      break;
    case fmi2_base_type_int:
      break;
    case fmi2_base_type_str:
      break;
    case fmi2_base_type_enum:
      break;
    }

  }

  ROS_DEBUG("Init done");
  adapter.exitInitializationMode(ros::Time::now());


  // ---
  // Spinning Loop
  // ---

  double updatePeriod = 0.01;  // Default is 0.01s
  n.getParam("update_period", updatePeriod);

  ros::Timer timer = n.createTimer(ros::Duration(updatePeriod), [&](const ros::TimerEvent& event) {
    if (adapter.getSimulationTime() < event.current_expected) {
      adapter.doStepsUntil(event.current_expected);
    } else {
      ROS_INFO("Simulation time %f is greater than timer's time %f. Is your step size to large?",
               adapter.getSimulationTime().toSec(), event.current_expected.toSec());
    }
    for (auto const& element : adapter.getOutputVariableNamesAndBaseTypes()) {

      switch(element.second) {
      case fmi2_base_type_real:
      {
          std_msgs::Float64 msg;
          msg.data = adapter.getOutputValue(element.first);
          publishers[element.first].publish(msg);
          break;
      }
      case fmi2_base_type_bool:
      {
          std_msgs::Bool msg;
          msg.data = adapter.getOutputValue(element.first);
          publishers[element.first].publish(msg);
          break;
      }
      case fmi2_base_type_int:
      {
        // TODO
        break;
      }
      case fmi2_base_type_str:
      {
        // TODO
        break;
      }
      case fmi2_base_type_enum:
      {
        // TODO
        break;
      }
      }

    }
  });

  ros::spin();

  return 0;
}
