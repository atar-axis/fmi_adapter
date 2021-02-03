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

#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/algorithm/copy.hpp>


namespace fmi_adapter {

class FMIMaster {
 private:
  std::map<std::string, std::unique_ptr<FMU>> slave_fmus{};

  std::map<std::pair<std::string, std::string>, std::shared_ptr<FMUVariable>> master_inputs{};
  std::map<std::pair<std::string, std::string>, std::shared_ptr<FMUVariable>> master_outputs{};

  double stepSize;

  nlohmann::json jsonConfig{};

  void propagateResults() {
    for (auto source : jsonConfig["connections"]) {
      auto fmuName = source[0].get<std::string>();

      // const auto signalName = fmuName.substr(0, fmuName.find('.'));
      // const auto result = slave_fmus[signalName]->getValue();

      for (auto sink : source) {
        // ROS_WARN("* to: %s", sink.get<std::string>().c_str());
      }
    }
  }


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
    json_stream >> jsonConfig;

    std::string dumped = jsonConfig["connections"].dump();
    ROS_WARN("connecting the slave in/outputs as follows: %s", dumped.c_str());


    for (auto [exposedFmu, exposedFmuSignals] : jsonConfig["expose"].items()) {
      ROS_WARN("exposing from fmu: %s", exposedFmu.c_str());
      for (auto signalName : exposedFmuSignals) {
        ROS_WARN("* signal: %s", signalName.get<std::string>().c_str());

        ROS_WARN("get variable");
        std::shared_ptr variable = slave_fmus.at(exposedFmu)->getCachedVariable(signalName);

        ROS_WARN("get causality of variable");
        auto causality = variable->getCausalityRaw();

        ROS_WARN("switching on causality of variable");
        switch (causality) {
          case fmi2_causality_enu_input:
            ROS_WARN("forwarding input");
            master_inputs.insert(std::make_pair(std::make_pair(exposedFmu, signalName.get<std::string>()), variable));
            break;
          case fmi2_causality_enu_output:
            ROS_WARN("forwarding output");
            master_outputs.insert(std::make_pair(std::make_pair(exposedFmu, signalName.get<std::string>()), variable));
            break;
          default:
            ROS_WARN("cannot expose variables other than input or output");
        }
      }
    }
  }

  void initSlavesFromROS(const ros::NodeHandle& wrappingNode) {
    for (auto& [fmuname, fmuptr] : slave_fmus) {
      (void)fmuname;  // variable 'name' is currently unused

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

    propagateResults();
  }

  const std::map<std::pair<std::string, std::string>, std::shared_ptr<FMUVariable>>& getOutputs() {
    return master_outputs;
  }

  const std::map<std::pair<std::string, std::string>, std::shared_ptr<FMUVariable>>& getInputs() {
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