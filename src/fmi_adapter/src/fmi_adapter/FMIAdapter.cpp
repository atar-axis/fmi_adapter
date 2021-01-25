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

#include "fmi_adapter/FMIAdapter.h"

#include <cstdlib>

#include <algorithm>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <utility>

#include <dirent.h>
#include <unistd.h>

#include <fmilib.h>

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


bool variableFilterAll(__attribute__((unused)) fmi2_import_variable_t* variable) { return true; }

bool variableFilterByCausality(fmi2_import_variable_t* variable, fmi2_causality_enu_t causality) {
  return (fmi2_import_get_causality(variable) == causality);
}

/**
 * @brief Get the Variables From FMU object as raw
 * This helper functions returns all variables with a certain property defined by an optional filter.
 *
 * @param fmu
 * @param filter
 * @return std::vector<fmi2_import_variable_t*>
 */
std::vector<fmi2_import_variable_t*> getVariablesFromFMURaw(
    fmi2_import_t* fmu,
    std::function<bool(fmi2_import_variable_t*)> filter = variableFilterAll
) {
  assert(fmu);

  std::vector<fmi2_import_variable_t*> result;

  fmi2_import_variable_list_t* variableList = fmi2_import_get_variable_list(fmu, 0);
  const size_t variablesCount = fmi2_import_get_variable_list_size(variableList);
  for (size_t index = 0; index < variablesCount; ++index) {
    fmi2_import_variable_t* variable = fmi2_import_get_variable(variableList, index);
    if (filter(variable)) {
      result.push_back(variable);
    }
  }

  fmi2_import_free_variable_list(variableList);

  return result;
}

/**
 * @brief Get the Variables From FMU as <Name, Type> map
 * This helper functions returns all variable names with a certain property defined by an optional filter.
 *
 * @param fmu
 * @param filter
 * @return std::map<std::string, fmi2_base_type_enu_t>
 */
std::map<std::string, fmi2_base_type_enu_t> getVariablesFromFMU(
    fmi2_import_t* fmu,
    std::function<bool(fmi2_import_variable_t*)> filter = variableFilterAll
) {
  //ROS_DEBUG("Start getVariablesFromFMU");

  assert(fmu);
  //ROS_DEBUG("fmu assert done");

  std::map<std::string, fmi2_base_type_enu_t> result;
  //ROS_DEBUG("map creation done");

  fmi2_import_variable_list_t* variableList = fmi2_import_get_variable_list(fmu, 0);
  const size_t variablesCount = fmi2_import_get_variable_list_size(variableList);
  //ROS_DEBUG("get fmi variables list and size done");

  for (size_t index = 0; index < variablesCount; ++index) {
    fmi2_import_variable_t* variable = fmi2_import_get_variable(variableList, index);
    //ROS_DEBUG("get fmi variable done");

    if (filter(variable)) {
      std::string name = fmi2_import_get_variable_name(variable);
      //ROS_DEBUG("get fmi variable name done: %s", name.c_str());
      fmi2_base_type_enu_t type = fmi2_import_get_variable_base_type(variable);
      //ROS_DEBUG("get fmi variable type done: %s", fmi2_base_type_to_string(type));
      //ROS_DEBUG("adding variable %s of base type %s", name.c_str(), fmi2_base_type_to_string(type));

      result[name] = type;
    }
  }

  fmi2_import_free_variable_list(variableList);

  //ROS_DEBUG("End getVariablesFromFMU");

  return result;
}

}  // namespace helpers



/**
 * @brief Construction of a new FMIAdapter::FMIAdapter object
 *
 * @param fmuPath
 * @param stepSize
 * @param interpolateInput
 * @param tmpPath
 */
FMIAdapter::FMIAdapter(
  const std::string& fmuPath,
  ros::Duration stepSize,
  bool interpolateInput,
  const std::string& tmpPath)
  : fmuPath_(fmuPath),
    stepSize_(stepSize),
    interpolateInput_(interpolateInput),
    tmpPath_(tmpPath)
{
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
}

/**
 * @brief Dectruction of a FMIAdapter::FMIAdapter object
 *
 */
FMIAdapter::~FMIAdapter() {
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
std::string FMIAdapter::rosifyName(const std::string& name) {
  std::string result = name;
  for (size_t i = 0; i < result.size(); ++i) {
    char c = result[i];
    if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || ('0' <= c && c <= '9') || c == '_') {
      // Keep valid char
    } else {
      result[i] = '_';
    }
  }

  while (result.length() > 0 && result[0] == '_') {
    result.erase(0, 1);
  }

  return result;
}


bool FMIAdapter::canHandleVariableCommunicationStepSize() const {
  return static_cast<bool>(fmi2_import_get_capability(fmu_, fmi2_cs_canHandleVariableCommunicationStepSize));
}

ros::Duration FMIAdapter::getDefaultExperimentStep() const {
  return ros::Duration(fmi2_import_get_default_experiment_step(fmu_));
}


std::vector<fmi2_import_variable_t*> FMIAdapter::getAllVariablesRaw() const {
  return helpers::getVariablesFromFMURaw(fmu_, helpers::variableFilterAll);
}

std::vector<fmi2_import_variable_t*> FMIAdapter::getInputVariablesRaw() const {
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1, fmi2_causality_enu_input);
  return helpers::getVariablesFromFMURaw(fmu_, filter);
}

std::vector<fmi2_import_variable_t*> FMIAdapter::getOutputVariablesRaw() const {
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1, fmi2_causality_enu_output);
  return helpers::getVariablesFromFMURaw(fmu_, filter);
}

std::vector<fmi2_import_variable_t*> FMIAdapter::getParametersRaw() const {
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1, fmi2_causality_enu_parameter);
  return helpers::getVariablesFromFMURaw(fmu_, filter);
}


std::map<std::string, fmi2_base_type_enu_t> FMIAdapter::getAllVariableNamesAndBaseTypes() const {
  return helpers::getVariablesFromFMU(fmu_, helpers::variableFilterAll);
}

std::map<std::string, fmi2_base_type_enu_t> FMIAdapter::getInputVariableNamesAndBaseTypes() const {
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1, fmi2_causality_enu_input);
  return helpers::getVariablesFromFMU(fmu_, filter);
}

std::map<std::string, fmi2_base_type_enu_t> FMIAdapter::getOutputVariableNamesAndBaseTypes() const {
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1, fmi2_causality_enu_output);
  return helpers::getVariablesFromFMU(fmu_, filter);
}

std::map<std::string, fmi2_base_type_enu_t> FMIAdapter::getParameterNamesAndBaseTypes() const {
  ROS_DEBUG("Start getParameterNamesAndBaseTypes");
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1, fmi2_causality_enu_parameter);
  auto retVal = helpers::getVariablesFromFMU(fmu_, filter);
  ROS_DEBUG("End getParameterNamesAndBaseTypes");
  return retVal;
}

/**
 * @brief Exits the initialization mode
 * Moreover, starts the simulation of the wrapped FMU. Uses the given timestamp
 * as start time for the simulation. The FMU internally starts at time 0.
 *
 * All times passed to setValue(...) and doStep*(...) are translated correspondingly.
 *
 * @param simulationTime time where the simulation should start
 */
void FMIAdapter::exitInitializationMode(ros::Time simulationTime) {
  if (!inInitializationMode_) {
    throw std::runtime_error("FMU is no longer in initialization mode!");
  }

  fmi2_status_t fmiStatus = fmi2_import_exit_initialization_mode(fmu_);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_exit_initialization_mode failed!");
  }
  inInitializationMode_ = false;

  fmuTimeOffset_ = simulationTime - ros::Time(0.0);
  assert(fmuTime_ == 0.0);

  for (fmi2_import_variable_t* variable : getInputVariablesRaw()) {  // TODO(Ralph) Avoid creation of std::vector here.

    // Creating a reference to the map of time and value for the current input-variable
    std::map<ros::Time, variable_type>& inputValues = inputValuesByVariable_[variable];

    // If there is no value set or the time to begin with is set to a point earlier than those of the values,
    // then set at least the value for the time where the simulation starts
    if (inputValues.empty() || inputValues.begin()->first > simulationTime) {
      fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);

      variable_type value;
      fmi2_base_type_enu_t type = fmi2_import_get_variable_base_type(variable);

      switch(type) {
      case fmi2_base_type_real:
      {
        double tmp = 0.0;
        fmi2_import_get_real(fmu_, &valueReference, 1, &tmp);
        value = tmp;
        break;
      }
      case fmi2_base_type_int:
      {
        int32_t tmp = 0;
        fmi2_import_get_integer(fmu_, &valueReference, 1, &tmp);
        value = tmp;
        break;
      }
      case fmi2_base_type_bool:
      case fmi2_base_type_str:
      case fmi2_base_type_enum:
        break;
      }

      inputValues[simulationTime] = value;
    }
  }
}

void FMIAdapter::_doStep(const ros::Duration& stepSize) {

  for (fmi2_import_variable_t* variable : getInputVariablesRaw()) {  // TODO(Ralph) Avoid creation of std::vector here.

    std::map<ros::Time, variable_type>& inputValues = inputValuesByVariable_[variable];

    assert(!inputValues.empty() && (inputValues.begin()->first - fmuTimeOffset_).toSec() <= fmuTime_);
    while (inputValues.size() >= 2 && (std::next(inputValues.begin())->first - fmuTimeOffset_).toSec() <= fmuTime_) {
      inputValues.erase(inputValues.begin());
    }
    assert(!inputValues.empty() && (inputValues.begin()->first - fmuTimeOffset_).toSec() <= fmuTime_);

    variable_type value = inputValues.begin()->second;

    /* TODO: Interpolation
    if (interpolateInput_ && inputValues.size() > 1) {
      double t0 = (inputValues.begin()->first - fmuTimeOffset_).toSec();
      double t1 = (std::next(inputValues.begin())->first - fmuTimeOffset_).toSec();
      double weight = (t1 - fmuTime_) / (t1 - t0);
      variable_type x0 = value;
      variable_type x1 = std::next(inputValues.begin())->second;
      value = weight * x0 + (1.0 - weight) * x1;
    }
    */

    fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);
    fmi2_base_type_enu_t type = fmi2_import_get_variable_base_type(variable);

    switch(type) {
    case fmi2_base_type_real:
    {
      // pass the value to the variable referenced by valueReference
      ROS_INFO("1");
      double val = boost::get<double>(value);
      ROS_INFO("2");
      fmi2_import_set_real(fmu_, &valueReference, 1, &val);
      break;
    }
    case fmi2_base_type_int:
    {
      ROS_INFO("3");
      int val = boost::get<int32_t>(value);
      ROS_INFO("4");
      fmi2_import_set_integer(fmu_, &valueReference, 1, &val);
      break;
    }
    case fmi2_base_type_bool:
    case fmi2_base_type_str:
    case fmi2_base_type_enum:
      break;
    }

  }


  const fmi2_boolean_t doStep = fmi2_true;
  fmi2_status_t fmiStatus = fmi2_import_do_step(fmu_, fmuTime_, stepSize.toSec(), doStep);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_do_step failed!");
  }
  fmuTime_ += stepSize.toSec();
}

ros::Time FMIAdapter::doStep() {
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  _doStep(stepSize_);

  return getSimulationTimeInternal();
}

ros::Time FMIAdapter::doStep(const ros::Duration& stepSize) {
  if (stepSize <= ros::Duration(0.0)) {
    throw std::invalid_argument("Step size must be positive!");
  }
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  _doStep(stepSize);

  return getSimulationTimeInternal();
}

ros::Time FMIAdapter::doStepsUntil(const ros::Time& simulationTime) {
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  fmi2_real_t targetFMUTime = (simulationTime - fmuTimeOffset_).toSec();
  if (targetFMUTime < fmuTime_ - stepSize_.toSec() / 2.0) {  // Subtract stepSize/2 for rounding.
    ROS_ERROR("Given time %f is before current simulation time %f!", targetFMUTime, fmuTime_);
    throw std::invalid_argument("Given time is before current simulation time!");
  }

  while (fmuTime_ + stepSize_.toSec() / 2.0 < targetFMUTime) {
    _doStep(stepSize_);
  }

  return getSimulationTimeInternal();
}


ros::Time FMIAdapter::getSimulationTime() const {
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  return getSimulationTimeInternal();
}


/**
 * @brief Set the value for a specific FMU input variable at a given time (e.g. now)
 * This Function is currently used internally
 *
 * @param variable
 * @param time
 * @param value
 */
void FMIAdapter::_setInputValueRaw(fmi2_import_variable_t* variable, ros::Time time, variable_type value) {
  if (fmi2_import_get_causality(variable) != fmi2_causality_enu_input) {
    throw std::invalid_argument("Given variable is not an input variable!");
  }

  std::string name = rosifyName(fmi2_import_get_variable_name(variable));
  ROS_INFO("adding a new input value for variable %s", name.c_str());
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
void FMIAdapter::setInputValue(std::string variableName, ros::Time time, variable_type value) {
  fmi2_import_variable_t* variable = fmi2_import_get_variable_by_name(fmu_, variableName.c_str());
  if (variable == nullptr) {
    throw std::invalid_argument("Unknown variable name!");
  }

  _setInputValueRaw(variable, time, value);
}




/**
 * @brief Get the current output for a specific variables name
 * Note: This Function is currently only used internally
 *
 * @param variable
 * @return double
 */
// TODO: UNUSED
// template <typename T>
// T FMIAdapter::_getOutputValueRaw(fmi2_import_variable_t* variable) const {
//   if (fmi2_import_get_causality(variable) != fmi2_causality_enu_output) {
//     throw std::invalid_argument("Given variable is not an output variable!");
//   }

//   fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);

//   T value;

//   // TODO: UNUSED
//   if (std::is_same<T, double>::value) {
//     fmi2_import_get_real(fmu_, &valueReference, 1, &value);
//   } else if (std::is_same<T, int>::value) {
//     fmi2_import_get_integer(fmu_, &valueReference, 1, &value);
//   } else if (std::is_same<T, bool>::value) {
//     fmi2_import_get_boolean(fmu_, &valueReference, 1, &value);
//   }

//   return value;
// }


template<typename T>
void fmi2_import_get_var(fmi2_import_t* fmu_, const fmi2_value_reference_t* valueReference, size_t nvr, T* value)
{
  ROS_WARN("The current variable was not handled correctly");
  return;
}

// Specializations, see: https://isocpp.org/wiki/faq/templates#template-specialization-piecemeal
template<>
void fmi2_import_get_var<double>(fmi2_import_t* fmu_, const fmi2_value_reference_t* valueReference, size_t nvr, double* value)
{
  //ROS_DEBUG("Setting a real/double variable");
  fmi2_import_get_real(fmu_, valueReference, nvr, value);
  return;
}

template<>
void fmi2_import_get_var<int>(fmi2_import_t* fmu_, const fmi2_value_reference_t* valueReference, size_t nvr, int* value)
{
  //ROS_DEBUG("Setting a int variable");
  fmi2_import_get_integer(fmu_, valueReference, nvr, value);
  return;
}

/**
 * @brief Get the current output for a specific variables name
 * This function is called inside the nodes spinning function to
 * propagate the output
 *
 * @tparam T
 * @param variableName
 * @return T
 */
template <typename T>
T FMIAdapter::getOutputValue(const std::string& variableName) const {

  fmi2_import_variable_t* variable = fmi2_import_get_variable_by_name(fmu_, variableName.c_str());
  fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);

  if (variable == nullptr) {
    throw std::invalid_argument("Unknown variable name!");
  }

  if (fmi2_import_get_causality(variable) != fmi2_causality_enu_output) {
    throw std::invalid_argument("Given variable is not an output variable!");
  }

  T value;
  fmi2_import_get_var<T>(fmu_, &valueReference, 1, &value);

  return value;

}
template double FMIAdapter::getOutputValue<double>(const std::string& variableName) const;
template int FMIAdapter::getOutputValue<int>(const std::string& variableName) const;




/**
 * @brief Initialize the FMU variables to a given value
 *
 * This function is used internally in order to initialize the FMU variables
 * by values from the ROS parameter server.
 *
 * @param variable
 * @param value
 */
void FMIAdapter::_setInitialValueRaw(fmi2_import_variable_t* variable, double value) {
  if (!inInitializationMode_) {
    throw std::runtime_error("Initial values can be only set in initialization mode!");
  }

  fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);
  fmi2_import_set_real(fmu_, &valueReference, 1, &value);
}

void FMIAdapter::_setInitialValueRaw(fmi2_import_variable_t* variable, int value) {
  if (!inInitializationMode_) {
    throw std::runtime_error("Initial values can be only set in initialization mode!");
  }

  fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);
  fmi2_import_set_integer(fmu_, &valueReference, 1, &value);
}


// TODO: Where is it used? Nowhere?
// void FMIAdapter::setInitialValue(const std::string& variableName, double value) {
//   fmi2_import_variable_t* variable = fmi2_import_get_variable_by_name(fmu_, variableName.c_str());
//   if (variable == nullptr) {
//     throw std::invalid_argument("Unknown variable name!");
//   }

//   setInitialValue(variable, value);
// }

/**
 * @brief Initializes the Adapter Node from ROS Parameters
 * This Function is reading out the Parameter Values from the ROS Parameter Server
 * and is setting every FMU variable accordingly.
 *
 * @param handle to the ROSNode
 */
void FMIAdapter::initializeFromROSParameters(const ros::NodeHandle& handle) {

  for (fmi2_import_variable_t* variable : helpers::getVariablesFromFMURaw(fmu_)) {

    std::string name = rosifyName(fmi2_import_get_variable_name(variable));
    fmi2_base_type_enu_t type = fmi2_import_get_variable_base_type(variable);

    // TODO: remove repetition

    switch(type) {
    case fmi2_base_type_real:
      {
        double valueStorage = 0.0;
        if (handle.getParam(name, valueStorage)){
          ROS_DEBUG("setting variable %s to its init value from ROS");
          _setInitialValueRaw(variable, valueStorage);
        }
      }
      break;
    case fmi2_base_type_int:
      {
        int valueStorage = 0;
        if (handle.getParam(name, valueStorage)){
          ROS_DEBUG("setting variable to its init value from ROS");
          _setInitialValueRaw(variable, valueStorage);
        }
      }
      break;
    case fmi2_base_type_bool:
          ROS_DEBUG("variables of type bool are currently not yet supported, cannot set");
    case fmi2_base_type_str:
          ROS_DEBUG("variables of type str are currently not yet supported, cannot set");
    case fmi2_base_type_enum:
          ROS_DEBUG("variables of type enum are currently not yet supported, cannot set");
      break;
    }
  }
}

}  // namespace fmi_adapter
