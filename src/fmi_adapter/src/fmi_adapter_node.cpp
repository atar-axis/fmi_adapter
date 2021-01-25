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
#include <std_msgs/Int32.h>
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
  fmi_adapter::FMIAdapter adapter(fmuPath, stepSize);

  for (auto const& element : adapter.getParameterNamesAndBaseTypes()) {
    ROS_DEBUG("FMU has parameter '%s'", element.first.c_str());
  }

  // Init the Adapter
  ROS_DEBUG("Initialize Adapter...");
  adapter.initializeFromROSParameters(n);


  ROS_DEBUG("Creating subscribers...");
  std::map<std::string, ros::Subscriber> subscribers;

  // Iterate over all FMU Input Variables and create subscribers for the wrapping node accordingly
  for (auto const& element : adapter.getInputVariableNamesAndBaseTypes()) {

    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(element.first);

    ros::Subscriber subscriber;

    switch(element.second) {
    case fmi2_base_type_real:
      subscriber = n.subscribe<std_msgs::Float64>(
        rosifiedName, 1000,
        [&adapter, element](const std_msgs::Float64::ConstPtr& msg) {
          std::string myName = element.first;
          variable_type value = msg->data;
          adapter.setInputValue(myName, ros::Time::now(), value);
        }
      );
      ROS_INFO("* created new subscriber for double variable: %s", rosifiedName.c_str());
      break;

    case fmi2_base_type_int:
      subscriber = n.subscribe<std_msgs::Int32>(
        rosifiedName, 1000,
        [&adapter, element](const std_msgs::Int32::ConstPtr& msg) {
          std::string myName = element.first;
          variable_type value = msg->data;
          adapter.setInputValue(myName, ros::Time::now(), value);
        }
      );
      ROS_INFO("* created new subscriber for int variable: %s", rosifiedName.c_str());
      break;
    case fmi2_base_type_bool:
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
  std::map<std::string, ros::Publisher> publishers;

  // Iterate over all FMU Output Variables and create publishers for the wrapping node accordingly
  for (auto const& element : adapter.getOutputVariableNamesAndBaseTypes()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(element.first);

    switch(element.second) {
    case fmi2_base_type_real:
      publishers[element.first] = n.advertise<std_msgs::Float64>(rosifiedName, 1000);
      ROS_INFO("* created new publisher for double variable: %s", rosifiedName.c_str());
      break;
    case fmi2_base_type_int:
      publishers[element.first] = n.advertise<std_msgs::Int32>(rosifiedName, 1000);
      ROS_INFO("* created new publisher for int variable: %s", rosifiedName.c_str());
      break;
    case fmi2_base_type_bool:
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

  double updatePeriod = 1.00;  // Default is 0.01s
  //TODO: set back!
  n.getParam("update_period", updatePeriod);

  ros::Timer timer = n.createTimer(ros::Duration(updatePeriod), [&](const ros::TimerEvent& event) {

    // simulate some steps
    if (adapter.getSimulationTime() < event.current_expected) {
      ROS_INFO("executing steps until %lld", (long long)event.current_expected.toNSec());
      adapter.doStepsUntil(event.current_expected);
    } else {
      ROS_INFO("Simulation time %f is greater than timer's time %f. Is your step size to large?",
               adapter.getSimulationTime().toSec(), event.current_expected.toSec());
    }

    // propagate the newly simulated output values
    for (auto const& element : adapter.getOutputVariableNamesAndBaseTypes()) {

      switch(element.second) {
      case fmi2_base_type_real:
      {
        std_msgs::Float64 msg;
        msg.data = adapter.getOutputValue<double>(element.first);
        publishers[element.first].publish(msg);
        break;
      }
      case fmi2_base_type_int:
      {
        std_msgs::Int32 msg;
        msg.data = adapter.getOutputValue<int>(element.first);
        publishers[element.first].publish(msg);
        break;
      }
      case fmi2_base_type_bool:
      {
        break;
      }
      case fmi2_base_type_str:
      {
        break;
      }
      case fmi2_base_type_enum:
      {
        break;
      }
      }

    }
  });

  ros::spin();

  return 0;
}
