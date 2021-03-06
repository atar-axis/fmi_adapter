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

#include "fmi_adapter/FMU.h"

#include <cstdlib>
#include <algorithm>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <utility>
#include <variant>

#include <dirent.h>
#include <unistd.h>

#include <fmilib.h>

#include <boost/regex.hpp>


// !!!!!!!   !!!!!   !!!!!!    !!!!!
//   !!!    !!! !!!  !!! !!!  !!! !!!
//   !!!    !!! !!!  !!! !!!  !!! !!!
//   !!!     !!!!!   !!!!!!    !!!!!

// TODO: * Add fmi2_base_type_str and enum?


namespace fmi_adapter {

namespace helpers {

bool canWriteToFolder(const std::string& path) {
  DIR* dir = opendir(path.c_str());
  if (dir == nullptr) {
    return false;
  }
  closedir(dir);
  return (access(path.c_str(), W_OK) == 0);
}

bool canReadFromFile(const std::string& path) {
  std::ifstream stream(path.c_str());
  if (stream.is_open() && stream.good()) {
    stream.close();
    return true;
  } else {
    return false;
  }
}

}  // namespace helpers


// Conversion: ROS Parameter <-> FMI

// unspecialized case: just cast it
template <typename Tin, typename Tout>
Tout FMU::convert(Tin value) {
  return (Tout)value;
}
// unspecialized cases: where casting doesn't work
template <>
fmi2_boolean_t FMU::convert<bool, fmi2_boolean_t>(bool value) {
  return value == true ? fmi2_true : fmi2_false;
}
template <>
bool FMU::convert<fmi2_boolean_t, bool>(fmi2_boolean_t value) {
  return value == fmi2_true ? true : false;
}
template <>
fmi2_boolean_t FMU::convert<uint8_t, fmi2_boolean_t>(uint8_t value) {
  return value == 1 ? fmi2_true : fmi2_false;
}
template <>
uint8_t FMU::convert<fmi2_boolean_t, uint8_t>(fmi2_boolean_t value) {
  return value == fmi2_true ? 1 : 0;
}

/**
 * @brief Constructor for a FMU::FMU object
 *
 * @param fmuPath
 * @param stepSize
 * @param interpolateInput
 * @param tmpPath
 */
FMU::FMU(const std::string& fmuName, const std::string& fmuPath, ros::Duration stepSize, bool interpolateInput,
         const std::string& tmpPath)
    : fmuName_(fmuName), fmuPath_(fmuPath), stepSize_(stepSize), interpolateInput_(interpolateInput),
      tmpPath_(tmpPath) {
  if (stepSize == ros::Duration(0.0)) {
    // Use step-size from FMU. See end of ctor.
  } else if (stepSize < ros::Duration(0.0)) {
    throw std::invalid_argument("Step size must be positive!");
  }
  if (!helpers::canReadFromFile(fmuPath)) {
    throw std::invalid_argument("Given FMU file '" + fmuPath + "' not found or cannot be read!");
  }
  if (tmpPath_.empty()) {
    char pathPattern[] = "/tmp/fmi_adapter_XXXXXX";
    tmpPath_ = mkdtemp(pathPattern);
    removeTmpPathInDtor_ = true;
  }
  if (!helpers::canWriteToFolder(tmpPath_)) {
    throw std::invalid_argument("Cannot access tmp folder '" + tmpPath_ + "'!");
  }

  // Some of the following lines have been taken from FMILibrary 2.3.0 src/test/fmi2_import_cs_test.c
  // under BSD style license.

  jmCallbacks_ = new jm_callbacks;
  jmCallbacks_->malloc = malloc;
  jmCallbacks_->calloc = calloc;
  jmCallbacks_->realloc = realloc;
  jmCallbacks_->free = free;
  jmCallbacks_->logger = jm_default_logger;
  jmCallbacks_->log_level = jm_log_level_error;
  jmCallbacks_->context = 0;

  context_ = fmi_import_allocate_context(jmCallbacks_);

  fmi_version_enu_t fmuVersion = fmi_import_get_fmi_version(context_, fmuPath_.c_str(), tmpPath_.c_str());
  if (fmuVersion != fmi_version_2_0_enu) {
    throw std::invalid_argument("Could not load the FMU or the FMU does not meet the FMI 2.0 standard!");
  }

  fmu_ = fmi2_import_parse_xml(context_, tmpPath_.c_str(), 0);
  if (!fmu_) {
    throw std::invalid_argument("Could not parse XML description of FMU!");
  }

  if (fmi2_import_get_fmu_kind(fmu_) != fmi2_fmu_kind_cs) {
    throw std::invalid_argument("Given FMU is not for co-simulation!");
  }

  fmi2_callback_functions_t* fmiCallbacks = new fmi2_callback_functions_t;
  fmiCallbacks->logger = fmi2_log_forwarding;
  fmiCallbacks->allocateMemory = calloc;
  fmiCallbacks->freeMemory = free;
  fmiCallbacks->componentEnvironment = fmu_;
  fmiCallbacks_ = fmiCallbacks;

  jm_status_enu_t jmStatus = fmi2_import_create_dllfmu(fmu_, fmi2_fmu_kind_cs, fmiCallbacks);
  if (jmStatus == jm_status_error) {
    throw std::runtime_error("Creation of dllfmu failed!");
  }

  const fmi2_string_t instanceName = fmi2_import_get_model_name(fmu_);
  const fmi2_string_t fmuLocation = nullptr;  // Indicates that FMU should get path to the unzipped location.
  const fmi2_boolean_t visible = fmi2_false;
  const fmi2_real_t relativeTol = 1e-4;
  jmStatus = fmi2_import_instantiate(fmu_, instanceName, fmi2_cosimulation, fmuLocation, visible);
  assert(jmStatus != jm_status_error);

  const fmi2_real_t startTime = 0.0;
  const fmi2_real_t stopTime = -1.0;
  fmi2_status_t fmiStatus = fmi2_import_setup_experiment(fmu_, fmi2_true, relativeTol, startTime, fmi2_false, stopTime);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_setup_experiment failed!");
  }

  fmiStatus = fmi2_import_enter_initialization_mode(fmu_);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_enter_initialization_mode failed!");
  }

  if (stepSize == ros::Duration(0.0)) {
    stepSize_ = ros::Duration(fmi2_import_get_default_experiment_step(fmu_));
    if (stepSize_ <= ros::Duration(0.0)) {
      throw std::invalid_argument("Default experiment step size from FMU is not positive!");
    }
    ROS_INFO("No step-size argument given. Using default from FMU, which is %fs.", stepSize_.toSec());
  }

  // caching the FMU variables once, just update them later on
  ROS_WARN("caching variables for fmu: %s", fmuName_.c_str());
  cacheVariables_();
}

/**
 * @brief Dectruction of a FMU::FMU object
 *
 */
FMU::~FMU() {
  fmi2_import_terminate(fmu_);
  fmi2_import_free_instance(fmu_);
  fmi2_import_destroy_dllfmu(fmu_);
  fmi2_import_free(fmu_);
  fmi_import_free_context(context_);
  delete jmCallbacks_;
  delete static_cast<fmi2_callback_functions_t*>(fmiCallbacks_);

  if (removeTmpPathInDtor_) {
    // TODO(Ralph) Remove folder fmi_adapter_XXXXXX from /tmp.
    // Such function is not provided by Posix or C++11/14.
    // Possibly use boost::filesystem::remove_all. Then other
    // filesystem functions used here and use of "/tmp" may be
    // replaced by corresponding functions from boost::filesystem.
  }
}


/**
 * @brief Rosify a given variable name
 * Converts a given variable name to the format used by ROS
 *
 * @param name
 * @return std::string
 */
std::string FMU::rosifyName(const std::string& name) {
  std::string result = name;

  boost::regex reInvalidCharacters("[^a-zA-Z0-9_]");
  result = boost::regex_replace(name, reInvalidCharacters, "_");

  return result;
}


bool FMU::canHandleVariableCommunicationStepSize() const {
  return static_cast<bool>(fmi2_import_get_capability(fmu_, fmi2_cs_canHandleVariableCommunicationStepSize));
}

ros::Duration FMU::getDefaultExperimentStep() const {
  return ros::Duration(fmi2_import_get_default_experiment_step(fmu_));
}


/**
 * @brief Exits the initialization mode
 * Moreover, starts the simulation of the wrapped FMU. Uses the given timestamp
 * as start time for the simulation. The FMU internally starts at time 0.
 *
 * All times passed to setValue(...) and doStep*(...) are translated correspondingly.
 *
 * @param externalStartTime time where the simulation starts (e.g. the current time from ROS)
 */
void FMU::exitInitializationMode(ros::Time externalStartTime) {
  if (!inInitializationMode_) {
    throw std::runtime_error("FMU is no longer in initialization mode!");
  }

  fmi2_status_t fmiStatus = fmi2_import_exit_initialization_mode(fmu_);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_exit_initialization_mode failed!");
  }
  inInitializationMode_ = false;

  // The ROS simulation time starts earlier than the FMUs internal time
  // because the FMUs are loaded a bit later. Therefore, remember the time
  // where it started in order to know how much to calculate later on
  // in doStepsUtil (since the ros time is passed, not the fmu time)
  rosStartTime_ = externalStartTime;

  // TODO: What is it good for? Document!
  for (auto& [name, variable] :
       cachedVariables_ | boost::adaptors::filtered(fmi_adapter::FMUVariable::varInput_filter)) {
    (void)name;

    std::map<ros::Time, valueVariantTypes>& inputValues = inputValuesByVariable_[variable->getVariablePointerRaw()];
    if (inputValues.empty() || inputValues.begin()->first > externalStartTime) {
      auto value = variable->getValue();
      //ROS_WARN("variable %s, idx: %d", variable->getNameRos().c_str(), variable->getValue().index());
      inputValues[externalStartTime] = value;
    }
  }

ROS_WARN("fmu is leaving init mode at %f", rosStartTime_.toSec());
}

void FMU::doStep_(const ros::Duration& stepSize) {
  assert(fmu_ != nullptr);

  for (auto& [name, variable] :
       cachedVariables_ | boost::adaptors::filtered(fmi_adapter::FMUVariable::varInput_filter)) {
    (void)name;
    std::map<ros::Time, valueVariantTypes>& inputValuesByTime =
        inputValuesByVariable_[variable->getVariablePointerRaw()];

    // Make sure that there are InputValues, at least for the simulation start
    assert(!inputValuesByTime.empty() && (inputValuesByTime.begin()->first - rosStartTime_).toSec() <= fmuTime_);

    // Remove Values in the past
    while (inputValuesByTime.size() >= 2 &&
           (std::next(inputValuesByTime.begin())->first - rosStartTime_).toSec() <= fmuTime_) {
      inputValuesByTime.erase(inputValuesByTime.begin());
    }

    // Make sure that there are STILL InputValues, at least for the simulation start
    assert(!inputValuesByTime.empty() && (inputValuesByTime.begin()->first - rosStartTime_).toSec() <= fmuTime_);

    valueVariantTypes value = inputValuesByTime.begin()->second;

    // TODO: Interpolation
    // if (interpolateInput_ && inputValues.size() > 1) {
    //   double t0 = (inputValues.begin()->first - rosStartTime_).toSec();
    //   double t1 = (std::next(inputValues.begin())->first - rosStartTime_).toSec();
    //   double weight = (t1 - fmuTime_) / (t1 - t0);
    //   valueVariantTypes x0 = value;
    //   valueVariantTypes x1 = std::next(inputValues.begin())->second;
    //   value = weight * x0 + (1.0 - weight) * x1;
    // }

    // ROS_WARN("setting value (variant index %d) of variable %s : %s...", value.index(), fmuPath_.c_str(), variable->getNameRos().c_str());
    variable->setValue(value);
    // ROS_WARN("done.");
  }

  const fmi2_boolean_t doStep = fmi2_true;
  // ROS_WARN("calling fmi2_import_do_step, current communication point: %f, stepsize: %f", fmuTime_, stepSize.toSec());
  fmi2_status_t fmiStatus = fmi2_import_do_step(fmu_, fmuTime_, stepSize.toSec(), doStep);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_do_step failed!");
  }
  fmuTime_ += stepSize.toSec();
  // ROS_WARN("done.");

}

ros::Time FMU::doStep() {
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  doStep_(stepSize_);

  return getSimTimeForROS_();
}

ros::Time FMU::doStep(const ros::Duration& stepSize) {
  if (stepSize <= ros::Duration(0.0)) {
    throw std::invalid_argument("Step size must be positive!");
  }
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  doStep_(stepSize);

  return getSimTimeForROS_();
}

ros::Time FMU::doStepsUntil(const ros::Time rosUntilTime) {
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  //ROS_WARN("doing steps in %s", this->fmuName_.c_str());

  // ROS_WARN("performing steps until %f, fmu leaved init mode at: %f ...", rosUntilTime.toSec(),
  // rosStartTime_.toSec());

  double targetFMUTime = (rosUntilTime - rosStartTime_).toSec();
  if (targetFMUTime < fmuTime_ - stepSize_.toSec() / 2.0) {  // Subtract stepSize/2 for rounding.
    ROS_ERROR("Given time %f is before current simulation time %f!", targetFMUTime, fmuTime_);
    throw std::invalid_argument("Given time is before current simulation time!");
  }

  while (fmuTime_ + stepSize_.toSec() / 2.0 < targetFMUTime) {
    // ROS_WARN("slave: doing a step until time is reached");

    doStep_(stepSize_);
  }

//ROS_WARN("done.");

  return getSimTimeForROS_();
}


ros::Time FMU::getSimulationTime() const {
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  return getSimTimeForROS_();
}


/**
 * @brief Set the value for a specific FMU input variable at a given time (e.g. now)
 * This Function is currently used internally
 *
 * @param variable
 * @param time
 * @param value
 */
void FMU::_setInputValueRaw(fmi2_import_variable_t* variable, ros::Time time, valueVariantTypes value) {
  if (fmi2_import_get_causality(variable) != fmi2_causality_enu_input) {
    throw std::invalid_argument("Given variable is not an input variable!");
  }

  std::string name = rosifyName(fmi2_import_get_variable_name(variable));

  inputValuesByVariable_[variable].insert(std::make_pair(time, value));
}

/**
 * @brief Set the value for a specific FMU input variable at a given time (e.g. now)
 * This function is called in a callback of a subscription to the inputs of the wrapping node
 * to propagate the values to the FMU
 *
 * @param variableName
 * @param time
 * @param value
 */
void FMU::setInputValue(std::string variableName, ros::Time time, valueVariantTypes value) {
  fmi2_import_variable_t* variable = fmi2_import_get_variable_by_name(fmu_, variableName.c_str());
  if (variable == nullptr) {
    throw std::invalid_argument("Unknown variable name!");
  }

  // ROS_WARN("adding input value (time: %f) for variable %s. active type: %d", time.toSec(), variableName.c_str(), value.index());
  _setInputValueRaw(variable, time, value);
}

/**
 * @brief Initializes the Adapter Node from ROS Parameters
 * This Function is reading out the Parameter Values from the ROS Parameter Server
 * and is setting every FMU variable accordingly.
 *
 * @param handle to the ROSNode
 */
void FMU::initializeFromROSParameters(const ros::NodeHandle& handle) {
  for (auto& [name, variable] : cachedVariables_) {
    (void)name;
    // get a representation of the current type
    auto variableValue = variable->getValue();

    // call ROS::getParam on the active type, store the result in variableValue, if any
    std::visit([handle, variable](auto& activeVariant) { handle.getParam(variable->getNameRos(), activeVariant); },
               variableValue);

    // write back the ROS param value to the variable
    variable->setValue(variableValue);
  }
}


/**
 * @brief Get the Variables From FMU object as raw
 * This helper functions returns all variables with a certain property defined by an optional filter.
 *
 */

void FMU::cacheVariables_() {
  cachedVariables_.clear();

  fmi2_import_variable_list_t* variableList = fmi2_import_get_variable_list(fmu_, 0);
  const size_t variablesCount = fmi2_import_get_variable_list_size(variableList);

  for (size_t index = 0; index < variablesCount; ++index) {
    fmi2_import_variable_t* variable = fmi2_import_get_variable(variableList, index);
    std::string variableName = std::string(fmi2_import_get_variable_name(variable));
    cachedVariables_.insert(std::make_pair(variableName, std::make_shared<FMUVariable>(fmu_, variable)));

// DEBUG START
// switch(fmi2_import_get_variable_base_type(variable)){
//   case fmi2_base_type_real:
//     ROS_WARN("added: var %s, type: real", variableName.c_str());
//     break;
//   case fmi2_base_type_int:
//     ROS_WARN("added: var %s, type: int", variableName.c_str());
//     break;
//   case fmi2_base_type_bool:
//     ROS_WARN("added: var %s, type: bool", variableName.c_str());
//     break;
//   case fmi2_base_type_enum:
//     ROS_WARN("added: var %s, type: enum", variableName.c_str());
//     break;
//   case fmi2_base_type_str:
//     ROS_WARN("added: var %s, type: string", variableName.c_str());
//     break;
// }
// DEBUG END
  }

  fmi2_import_free_variable_list(variableList);
  return;
}


std::map<std::string, std::shared_ptr<FMUVariable>> FMU::getCachedVariables() const {
  assert(fmu_);
  return cachedVariables_;
}

std::shared_ptr<FMUVariable> FMU::getCachedVariable(std::string name) {
  assert(fmu_);
  return cachedVariables_.at(name);
}

fmi2_import_t* FMU::getRawFMU() {
  assert(fmu_);
  return fmu_;
}

}  // namespace fmi_adapter
