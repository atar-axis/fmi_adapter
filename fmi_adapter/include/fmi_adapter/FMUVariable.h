/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <variant>

#include <ros/ros.h>
#include <fmilib.h>

#include <FMI2/fmi2_enums.h>
#include <FMI2/fmi2_functions.h>
#include <FMI2/fmi2_import_variable.h>


namespace fmi_adapter {

typedef std::variant<double, int, bool> valueVariantTypes;

class FMUVariable {
 private:
  fmi2_import_variable_t* variable;  // TODO: use an UUID instead
  fmi2_value_reference_t valueReference;
  std::string rawName;
  fmi2_base_type_enu_t rawType;
  fmi2_causality_enu_t rawCausality;

  fmi2_import_t* parent_fmu;

 public:
  FMUVariable(fmi2_import_t* parent_fmu, fmi2_import_variable_t* element);
  ~FMUVariable() = default;

  // no copies or assignments allowed, we are holding an pointer!
  FMUVariable(const FMUVariable&) = delete;
  FMUVariable& operator=(const fmi_adapter::FMUVariable&) = delete;

  std::string rosifyName(const std::string& rawName) const;

  // getters for non-changing variable attributes

  fmi2_import_variable_t* getVariablePointerRaw() const;
  std::string getNameRaw() const;
  fmi2_base_type_enu_t getTypeRaw() const;
  fmi2_causality_enu_t getCausalityRaw() const;
  std::string getNameRos() const;
  fmi2_value_reference_t getValueReference() const;

  // filters for use in boost::adaptors::filtered
  static bool varInput_filter(std::shared_ptr<FMUVariable> variable);
  static bool varOutput_filter(std::shared_ptr<FMUVariable> variable);
  static bool varParam_filter(std::shared_ptr<FMUVariable> variable);

  valueVariantTypes getValue();
  void setValue(valueVariantTypes value);
};

}  // namespace fmi_adapter