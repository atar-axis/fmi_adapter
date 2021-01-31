/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#pragma once

#include <map>
#include <string>
#include <variant>

#include "FMU.h"

#include <ros/ros.h>


namespace fmi_adapter {

class FMIMaster {
  std::map<std::string, std::unique_ptr<FMU>> slave_fmus{};

  std::map<std::string, std::shared_ptr<FMUVariable>> master_inputs{};
  std::map<std::string, std::shared_ptr<FMUVariable>> master_outputs{};

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
      ROS_WARN("The Slave FMU called %s already exists in this master! Not added!", unique_name.c_str());
      return;
    }

    // Create a new FMU and insert it into the list of slaves
    slave_fmus.insert(std::make_pair(unique_name, std::make_unique<FMU>(fmuPath, ros::Duration(stepSize))));
    ROS_INFO("Added new Slave FMU to the Master! Path: %s", fmuPath.c_str());
  }

  void config() {
    if (slave_fmus.empty()) {
      ROS_ERROR("Cannot config master. Add Slaves before configuring it!");
    }

    for (auto& [name, fmu] : slave_fmus) {
      auto allElements = fmu->getCachedVariables();

      // TODO: forward outputs from slaves to master (but not all)
      for (auto const output : allElements | boost::adaptors::filtered(fmi_adapter::FMUVariable::varOutput_filter)) {
        std::string outputName = name + "__" + output->getNameRaw();
        master_outputs.insert(std::make_pair(outputName, output));
      }

      // TODO: forward inputs from master to slaves (but not all)
      for (auto const input : allElements | boost::adaptors::filtered(fmi_adapter::FMUVariable::varInput_filter)) {
        std::string inputName = name + "__" + input->getNameRaw();
        master_inputs.insert(std::make_pair(inputName, input));
      }
    }
  }

  void initSlavesFromROS(const ros::NodeHandle& wrappingNode) {
    for (auto& [name, fmu] : slave_fmus) {
      (void)name;  // variable 'name' is currently unused

      // TODO: Is it really a good idea to simply pass the variable?
      // TODO: Maybe we should do some work here instead of doing it in the slaves?
      // TODO: What are those Parameters even? What are they good for?
      fmu->initializeFromROSParameters(wrappingNode);
    }
  }

  void exitInitModeSlaves(ros::Time simulationTime) {
    // Complete the Initialization, i.e. set the starttime
    for (auto& [name, fmu] : slave_fmus) {
      (void)name;  // variable 'name' is currently unused

      // TODO: Is it really a good idea to simply pass the variable?
      // TODO: Maybe we should do some work here instead of doing it in the slaves?
      fmu->exitInitializationMode(simulationTime);
    }
  }

  void doStepsUntil(const ros::Time& simulationTime) {
    for (auto& [name, fmu] : slave_fmus) {
      (void)name;  // variable 'name' is currently unused

      fmu->doStepsUntil(simulationTime);
    }

    // TODO: process as configured, i.e. store the results somewhere for the next step
    // ? When are those values exchanged between the slaves?! On every Step?
  }

  const std::map<std::string, std::shared_ptr<FMUVariable>>& getOutputs() {
    // TODO
    return master_outputs;
  }

  const std::map<std::string, std::shared_ptr<FMUVariable>>& getInputs() {
    // TODO
    return master_inputs;
  }

  void setInputValue() {
    // TODO: implement
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