/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#pragma once

#include <fstream>
#include <map>
#include <string>
#include <variant>

#include "FMU.h"

#include <ros/ros.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace fmi_adapter {

class FMIMaster {
  std::map<std::string, std::unique_ptr<FMU>> slave_fmus{};

  std::map<std::tuple<std::string, std::string>, std::shared_ptr<FMUVariable>> master_inputs{};
  std::map<std::tuple<std::string, std::string>, std::shared_ptr<FMUVariable>> master_outputs{};

  double stepSize;


 public:
  FMIMaster(double stepSize) : stepSize(stepSize) { ROS_INFO("FMI Master created"); }
  ~FMIMaster() = default;

  // Copy and assignments not allowed
  FMIMaster(const FMIMaster& other) = delete;
  FMIMaster& operator=(const FMIMaster&) = delete;

  // add a new FMU
  void createSlave(std::string unique_name, std::string fmuPath) {
    // Check if the name is really unique
    if (slave_fmus.find(unique_name) != slave_fmus.end()) {
      ROS_WARN("A slave named %s already exists in this master! Not added!", unique_name.c_str());
      return;
    }

    // Create a new FMU and insert it into the list of slaves
    slave_fmus.insert(std::make_pair(unique_name, std::make_unique<FMU>(fmuPath, ros::Duration(stepSize))));
    ROS_INFO("Added new Slave FMU to the Master! Path: %s", fmuPath.c_str());
  }

  void config(const std::string json_path) {
    if (slave_fmus.empty()) {
      ROS_ERROR("Cannot config master. Add Slaves before configuring it!");
    }

    std::ifstream json_stream(json_path);
    json j;
    json_stream >> j;


    for (const auto& [fmuname, fmuptr] : slave_fmus) {
      auto allElements = fmuptr->getCachedVariables();

      // TODO: forward outputs from slaves to master (but not all)
      for (auto const output : allElements | boost::adaptors::filtered(fmi_adapter::FMUVariable::varOutput_filter)) {
        // std::string outputName = "___" + name + "___" + output->getNameRaw();
        master_outputs.insert(std::make_pair(std::make_tuple(fmuname, output->getNameRaw()), output));
      }

      // TODO: forward inputs from master to slaves (but not all)
      for (auto const input : allElements | boost::adaptors::filtered(fmi_adapter::FMUVariable::varInput_filter)) {
        // std::string inputName = "___" + name + "___" + input->getNameRaw();
        master_inputs.insert(std::make_pair(std::make_tuple(fmuname, input->getNameRaw()), input));
      }
    }
  }

  void initSlavesFromROS(const ros::NodeHandle& wrappingNode) {
    for (auto& [fmuname, fmuptr] : slave_fmus) {
      (void)fmuname;  // variable 'name' is currently unused

      // TODO: Is it really a good idea to simply pass the variable?
      // TODO: Maybe we should do some work here instead of doing it in the slaves?
      // TODO: What are those Parameters even? What are they good for?
      fmuptr->initializeFromROSParameters(wrappingNode);
    }
  }

  void exitInitModeSlaves(ros::Time simulationTime) {
    // Complete the Initialization, i.e. set the starttime
    ROS_WARN("Exiting Init Mode ...");
    for (auto& [fmuname, fmuptr] : slave_fmus) {
      (void)fmuname;  // variable 'name' is currently unused

      // TODO: Is it really a good idea to simply pass the variable?
      // TODO: Maybe we should do some work here instead of doing it in the slaves?
      fmuptr->exitInitializationMode(simulationTime);
    }
    ROS_WARN("Exiting Init Mode done!");
  }

  void doStepsUntil(const ros::Time& simulationTime) {
    for (auto& [fmuname, fmuptr] : slave_fmus) {
      (void)fmuname;  // variable 'name' is currently unused

      fmuptr->doStepsUntil(simulationTime);
    }

    // TODO: process as configured, i.e. store the results somewhere for the next step
    // ? When are those values exchanged between the slaves?! On every Step?
  }

  const std::map<std::tuple<std::string, std::string>, std::shared_ptr<FMUVariable>>& getOutputs() {
    return master_outputs;
  }

  const std::map<std::tuple<std::string, std::string>, std::shared_ptr<FMUVariable>>& getInputs() {
    return master_inputs;
  }

  void setInputValue(std::string fmuName, std::string portName, ros::Time when, valueVariantTypes value) {
    slave_fmus[fmuName]->setInputValue(portName, when, value);
    return;
  }

  ros::Time getSimulationTime() const {
    // TODO: hold your own masterTime instead of returning an arbitrary fmus one
    if (slave_fmus.empty()) {
      ROS_ERROR("No Slave, no Time!");
      throw;
    }

    return slave_fmus.begin()->second->getSimulationTime();
  };
};

}  // namespace fmi_adapter