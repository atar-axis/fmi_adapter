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

#pragma once

#include <cassert>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "FMUVariable.h"

#include <ros/ros.h>

#include <FMI2/fmi2_enums.h>
#include <FMI2/fmi2_functions.h>

#include <boost/range/adaptor/filtered.hpp>


struct fmi_xml_context_t;
typedef struct fmi_xml_context_t fmi_import_context_t;
struct fmi2_import_t;
struct fmi2_xml_variable_t;
typedef struct fmi2_xml_variable_t fmi2_import_variable_t;

struct jm_callbacks;


namespace fmi_adapter {

// An instance of this class wraps a FMU and allows to simulate it using the ROS time notion and standard C++ types.
// In the background, the FMI Library (a BSD-licensed C library) is used for interacting with the FMU.
// This class also provides concenvience functions to read parameters and initial values from ROS parameters.
class FMU {
 public:
  // This ctor creates an instance using the FMU from the given path. If the step-size argument
  // is zero, the default experiment step-size given in the FMU is used.
  explicit FMU(const std::string& fmuPath, ros::Duration stepSize = ros::Duration(0.0), bool interpolateInput = true,
               const std::string& tmpPath = "");

  FMU(const FMU& other) = delete;
  FMU& operator=(const FMU&) = delete;

  virtual ~FMU();

  // Returns a ROSified version of the given variable name, replaces all not supported characters with '_'.
  static std::string rosifyName(const std::string& name);

  // Returns true if the FMU of this instances supports a variable communication step-size.
  bool canHandleVariableCommunicationStepSize() const;

  // Returns the default experiment step-size of the FMU of this instance.
  ros::Duration getDefaultExperimentStep() const;


  // Stores a value for the given variable to be considered by doStep*(..) at the given time of the FMU simulation.
  void _setInputValueRaw(fmi2_import_variable_t* variable, ros::Time time, valueVariantTypes value);

  // Stores a value for the variable with the given name to be considered by doStep*(..) at the given
  // time of the FMU simulation.
  void setInputValue(std::string variableName, ros::Time time, valueVariantTypes value);

  // Returns the step-size used in the FMU simulation.
  ros::Duration getStepSize() const { return stepSize_; }

  // Returns true if the wrapped FMU is still in initialization mode, which allows to set parameters
  // and initial values.
  bool isInInitializationMode() const { return inInitializationMode_; }

  // Exits the initialization mode and starts the simulation of the wrapped FMU. Uses the given timestamp
  // as start time for the simulation whereas the FMU internally starts at time 0.
  // All times passed to setValue(..) and doStep*(..) are translated correspondingly.
  void exitInitializationMode(ros::Time simulationTime);

  // Performs one simulation step using the configured step size and returns the current simulation time.
  ros::Time doStep();

  // Performs one simulation step using the given step size and returns the current simulation time.
  ros::Time doStep(const ros::Duration& stepSize);

  // Advances the simulation of the wrapped FMU until the given point in time (modulo step-size).
  // In detail, the simulation is performed iteratively using the configured step-size. Before each simulation step
  // the relevant input values passed previously by setInputValue(..) are set depending on the given timestamps.
  ros::Time doStepsUntil(const ros::Time& simulationTime);

  // Returns the current simulation time.
  ros::Time getSimulationTime() const;

  // Tries to read inital values for each variable (including parameters and aliases) from the ROS parameter set.
  // The function only considers private ROS parameters, i.e. parameters that have the node name as prefix.
  // Note that ROS parameter names may use the characters [A-Za-z0-9_] only. Therefore, all other characters in an
  // FMU variable name are mapped to '_'. For example, to pass a value for the FMU variable 'dx[2]', use the ROS
  // parameter '/my_node_name/dx_2_'.
  void initializeFromROSParameters(const ros::NodeHandle& handle);

  std::map<std::string, std::shared_ptr<FMUVariable>> getCachedVariables() const;
  std::shared_ptr<FMUVariable> getCachedVariable(std::string name);

  // variable type conversion helpers
  template <typename Tin, typename Tout>
  static Tout convert(Tin value);

  fmi2_import_t* getRawFMU();

 private:
  // Path of the FMU being wrapped by this instance.
  const std::string fmuPath_;
  // Step size for the FMU simulation
  ros::Duration stepSize_;
  // Interpolate the input signals (as continuous), or not (piecewise constant)
  bool interpolateInput_{false};
  // Path to folder for temporary extraction of the FMU archive
  std::string tmpPath_;
  // If given tmp path is "", then an arbitrary one in /tmp is used. Remove this folder
  bool removeTmpPathInDtor_{false};
  // Is the FMU is still in init mode
  bool inInitializationMode_{true};

  // The internal FMU simulation time
  double fmuTime_{0.0};
  // Offset between the FMU's time and the ROS simulation time, used for doStep*(..) and setValue(..)
  ros::Duration rosTimeOffset_{0.0};
  // Stores the FMU Variables
  std::map<std::string, std::shared_ptr<FMUVariable>> cachedVariables_{};
  // Stores the mapping from timestamps to variable values for the FMU simulation
  std::map<fmi2_import_variable_t*, std::map<ros::Time, valueVariantTypes>> inputValuesByVariable_{};

  // Pointer from the FMU Library types to the FMU instance
  fmi2_import_t* fmu_{nullptr};
  // Pointer from the FMU Library types to the FMU context
  fmi_import_context_t* context_{nullptr};
  // Callback functions for FMI Library. Cannot use type fmi2_callback_functions_t here as it is a typedef of an
  // anonymous struct, cf. https://stackoverflow.com/questions/7256436/forward-declarations-of-unnamed-struct
  void* fmiCallbacks_{nullptr};
  // Further callback functions for FMI Library
  jm_callbacks* jmCallbacks_{nullptr};

  // Performs one simulation step using the given step size. Argument and initialization mode not checked.
  void doStep_(const ros::Duration& stepSize);
  // Returns the current simulation time w.r.t. the time where the FMU simulation started in ROS.
  ros::Time getSimTimeForROS_() const { return ros::Time(fmuTime_) + rosTimeOffset_; }
  // Fill the FMU Variables Map
  void cacheVariables_();
};

}  // namespace fmi_adapter
