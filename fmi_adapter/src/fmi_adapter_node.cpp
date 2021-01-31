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
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include "fmi_adapter/FMIMaster.h"


// TODO: * This is the only place where ROS:: elements should be used,
// TODO:   remove them everywhere else


template <typename T>
struct toRosMsg;
template <typename T>
using toRosMsg_t = typename toRosMsg<T>::type;

template <>
struct toRosMsg<int> {
  using type = std_msgs::Int32;
};
template <>
struct toRosMsg<double> {
  using type = std_msgs::Float64;
};
template <>
struct toRosMsg<bool> {
  using type = std_msgs::Bool;
};


int main(int argc, char** argv) {
  ROS_INFO("> Creating ROS node...");
  ros::init(argc, argv, "fmi_adapter_node");
  ros::NodeHandle n("~");

  ROS_INFO("> Reading in ROS node parameters...");

  std::string fmuPath;
  if (!n.getParam("fmu_path", fmuPath)) {
    ROS_ERROR("Parameter 'fmu_path' not specified!");
    throw std::runtime_error("Parameter 'fmu_path' not specified!");
  }

  double stepSizeAsDouble = 0.0;
  n.getParam("step_size", stepSizeAsDouble);
  ros::Duration stepSize(stepSizeAsDouble);

  double updatePeriod = 0.01;  // Default is 0.01s
  n.getParam("update_period", updatePeriod);


  auto master = fmi_adapter::FMIMaster(stepSizeAsDouble);
  master.createSlave("testUnit1", fmuPath);
  master.initSlavesFromROS(n);
  master.config();  // ! dummy (pass some config informations here, e.g. a json file)

  ROS_WARN("master inputs:");
  std::map<std::string, ros::Subscriber> subscribers;
  auto masterIn = master.getInputs();

  for (auto const& [name, element] : masterIn) {
    ROS_WARN("* %s", name.c_str());

    // get a empty dummy value from the variable (which is a std::variant) to deduce the type of it
    // then subscribe to a topic and forward incoming messages to the master
    // remember: ros is using different types in comparison to the FMU, therefore we need to translate those
    auto elVal = element->getValue();
    std::visit(
        [&](auto& activeVariant) {
          using variantType = typename std::remove_reference<decltype(activeVariant)>::type;
          using rosMsgType = toRosMsg_t<variantType>;
          subscribers[name] =
              n.subscribe<rosMsgType>(name, 1000, [&master, &name](const typename rosMsgType::ConstPtr& msg) {
                master.setInputValue(name, ros::Time::now(), (variantType)msg->data);
              });
        },
        elVal);
  }

  ROS_WARN("master outputs:");
  std::map<std::string, ros::Publisher> publishers;
  auto masterOut = master.getOutputs();

  // Create a publisher topic for every output
  for (auto const& [name, element] : masterOut) {
    ROS_WARN("* %s", name.c_str());

    auto elVal = element->getValue();
    std::visit(
        [&](auto& activeVariant) {
          using variantType = typename std::remove_reference<decltype(activeVariant)>::type;
          using rosMsgType = toRosMsg_t<variantType>;
          publishers[name] = n.advertise<rosMsgType>(name, 1000);
        },
        elVal);
  }

  master.exitInitModeSlaves(ros::Time::now());

  ROS_WARN("creating timer...");

  //! You need to assign the created timer to an variable - why?
  auto timer = n.createTimer(ros::Duration(updatePeriod), [&](const ros::TimerEvent& event) {
    // simulate steps until we reach the desired time
    if (master.getSimulationTime() < event.current_expected) {
      master.doStepsUntil(event.current_expected);
    } else {
      ROS_WARN("Simulation time %f is greater than timer's time %f. Is your step size to large?",
               master.getSimulationTime().toSec(), event.current_expected.toSec());
    }

    // propagate the output values to the publishers
    for (auto const& [name, element] : masterOut) {
      auto elVal = element->getValue();
      std::visit(
          [&](auto& activeVariant) {
            using variantType = typename std::remove_reference<decltype(activeVariant)>::type;
            using rosMsgType = toRosMsg_t<variantType>;

            rosMsgType msg;
            msg.data = activeVariant;
            publishers[name].publish(msg);
          },
          elVal);
    }
  });

  ros::spin();

  return 0;
}
