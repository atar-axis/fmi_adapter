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

#include <boost/filesystem.hpp>  // for path

namespace fmi_adapter {

class FMIMaster {
 private:
  std::map<std::string, std::unique_ptr<FMU>> slave_fmus{};

  std::map<std::pair<std::string, std::string>, std::shared_ptr<FMUVariable>> master_inputs{};
  std::map<std::pair<std::string, std::string>, std::shared_ptr<FMUVariable>> master_outputs{};

  double stepSize;

  nlohmann::json jsonConfig{};

  void propagateResults() {
    ROS_WARN("propagate results ...");
    try {
      for (auto [sourceName, sinkArray] : jsonConfig["connections"].items()) {
        const auto sourceName_fmu = sourceName.substr(0, sourceName.find('.'));
        const auto sourceName_signal = sourceName.substr(sourceName.find('.') + 1);
        const auto result = slave_fmus[sourceName_fmu]->getCachedVariable(sourceName_signal)->getValue();

        for (auto sinkName : sinkArray) {
          const auto sinkName_fmu = sinkName.get<std::string>().substr(0, sinkName.get<std::string>().find('.'));
          const auto sinkName_signal = sinkName.get<std::string>().substr(sinkName.get<std::string>().find('.') + 1);
              ROS_WARN("b");
          slave_fmus[sinkName_fmu]->setInputValue(sinkName_signal, ros::Time::now(), result);
        }
      }
    } catch (const std::out_of_range& oor) {
      ROS_FATAL(
          "There seems to be a problem with your inter-component connections. "
          "Please make sure you named them as in the FMUS.\n Error: %s", oor.what());
    } catch (...) {
      ROS_FATAL("Unknown Error while propagating the results.");
    }
    ROS_WARN("done.");
  }

  void createSlave(std::string unique_name, std::string fmuPath) {
    // Check if the name is really unique
    if (slave_fmus.count(unique_name) != 0) {
      ROS_WARN("A slave named %s already exists in this master but names need to be unique (not added therefore)!", unique_name.c_str());
      return;
    }

    // Create a new FMU and insert it into the list of slaves
    slave_fmus.insert(std::make_pair(unique_name, std::make_unique<FMU>(unique_name, fmuPath, ros::Duration(stepSize))));
    ROS_INFO("Added new Slave FMU to the Master! Path: %s", fmuPath.c_str());
  }

 public:
  FMIMaster(double stepSize) : stepSize(stepSize) { ROS_INFO("FMI Master created"); }
  ~FMIMaster() = default;

  // Copy and assignments not allowed
  FMIMaster(const FMIMaster& other) = delete;
  FMIMaster& operator=(const FMIMaster&) = delete;

  void config(const std::string json_path) {
    std::ifstream json_stream(json_path);
    json_stream >> jsonConfig;

    auto fmuEntries = jsonConfig["fmus"];
    if (fmuEntries.empty()) ROS_FATAL("Error! No FMUs specified in the config file");

    for (auto [fmuName, fmuPathRelative] : fmuEntries.items()) {
      boost::filesystem::path canonicalFmuPath = boost::filesystem::canonical(
          boost::filesystem::path(fmuPathRelative), boost::filesystem::path(json_path).parent_path());
      ROS_WARN("adding new slave %s: %s", fmuName.c_str(), canonicalFmuPath.string().c_str());
      createSlave(fmuName, canonicalFmuPath.string());
    }


    auto exposeEntries = jsonConfig["expose"];
    if (exposeEntries.empty()) ROS_WARN("Warning! No slave signals are exposed, seems a bit useless?");


    for (auto exposedEntry : exposeEntries) {


        auto signalDelimiter = exposedEntry.get<std::string>().find(".");
        auto exposedFmu = exposedEntry.get<std::string>().substr(0, signalDelimiter);
        auto signalName = exposedEntry.get<std::string>().substr(signalDelimiter + 1);

        try {
          std::shared_ptr variable = slave_fmus.at(exposedFmu)->getCachedVariable(signalName);

          switch (variable->getCausalityRaw()) {
            case fmi2_causality_enu_input:
              master_inputs.insert(std::make_pair(std::make_pair(exposedFmu, signalName), variable));
              break;
            case fmi2_causality_enu_output:
              master_outputs.insert(std::make_pair(std::make_pair(exposedFmu, signalName), variable));
              break;
            default:
              ROS_WARN("cannot expose variables other than input or output");
          }
          ROS_FATAL("exposed %s.%s", exposedFmu.c_str(), signalName.c_str());
        } catch (...) {
          ROS_FATAL("error while exposing %s.%s", exposedFmu.c_str(), signalName.c_str());
        }
    }

    // TODO: Check connections between FMUS (names, types)

  }

  void initSlavesFromROS(const ros::NodeHandle& wrappingNode) {
    for (auto& [fmuname, fmuptr] : slave_fmus) {
      (void)fmuname;  // variable 'name' is currently unused

      fmuptr->initializeFromROSParameters(wrappingNode);
    }
  }

  void exitInitModeSlaves(ros::Time simulationTime) {
    // Complete the Initialization, i.e. set the starttime
    for (auto& [fmuname, fmuptr] : slave_fmus) {
      (void)fmuname;  // variable 'name' is currently unused

      // TODO: Is it really a good idea to simply pass the variable?
      // TODO: Maybe we should do some work here instead of doing it in the slaves?
      fmuptr->exitInitializationMode(simulationTime);
    }
    ROS_WARN("Exiting Init Mode done!");
  }

  void doStepsUntil(const ros::Time simulationTime) {
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
    ROS_WARN("a");
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