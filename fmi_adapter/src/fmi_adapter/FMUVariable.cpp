/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#include <fmi_adapter/FMIAdapter.h>
#include <fmi_adapter/FMUVariable.h>

namespace fmi_adapter {


bool FMUVariable::varInput_filter(std::shared_ptr<FMUVariable> variable) {
  return variable->getCausalityRaw() == fmi2_causality_enu_input;
}

bool FMUVariable::varOutput_filter(std::shared_ptr<FMUVariable> variable) {
  return variable->getCausalityRaw() == fmi2_causality_enu_output;
}

bool FMUVariable::varParam_filter(std::shared_ptr<FMUVariable> variable) {
  return variable->getCausalityRaw() == fmi2_causality_enu_parameter;
}

std::string FMUVariable::rosifyName(const std::string& rawName) const {
  std::string result = rawName;
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

fmi2_import_variable_t* FMUVariable::getVariablePointerRaw() const { return variable; }
std::string FMUVariable::getNameRaw() const { return rawName; }
fmi2_base_type_enu_t FMUVariable::getTypeRaw() const { return rawType; }
fmi2_causality_enu_t FMUVariable::getCausalityRaw() const { return rawCausality; }
std::string FMUVariable::getNameRos() const { return rosifyName(rawName); }
fmi2_value_reference_t FMUVariable::getValueReference() const { return valueReference; }


FMUVariable::FMUVariable(fmi2_import_t* parent_fmu_passed, fmi2_import_variable_t* element)
    : variable(element), valueReference(fmi2_import_get_variable_vr(element)),
      rawName(std::string(fmi2_import_get_variable_name(element))),
      rawType(fmi2_import_get_variable_base_type(element)), rawCausality(fmi2_import_get_causality(element)),
      parent_fmu(parent_fmu_passed) {}

valueVariantTypes FMUVariable::getValue() {
  // ! not really necessary and it is preventing reading it out internally (to deduce the type)
  // if (rawCausality != fmi2_causality_enu_output) {
  //   throw std::invalid_argument("Given variable is not an output variable!");
  // }

  valueVariantTypes ret;

  switch (rawType) {
    case fmi2_base_type_real: {
      fmi2_real_t value = 0.0;
      fmi2_import_get_real(parent_fmu, &valueReference, 1, &value);
      ret = (double)value;
      break;
    }
    case fmi2_base_type_int: {
      fmi2_integer_t value = 0;
      fmi2_import_get_integer(parent_fmu, &valueReference, 1, &value);
      ret = (int)value;
      break;
    }
    case fmi2_base_type_bool: {
      fmi2_boolean_t value = fmi2_false;
      fmi2_import_get_boolean(parent_fmu, &valueReference, 1, &value);
      ret = (bool)(value == fmi2_true ? true : false);
      break;
    }
    default:
      break;
  }

  return ret;
}

void FMUVariable::setValue(valueVariantTypes values) {
  // TODO: Do not pass the initMode but get it from the parent
  // if (!initMode) {
  //   throw std::runtime_error("Initial values can be only set in initialization mode!");
  // }

  switch (rawType) {
    case fmi2_base_type_real: {
      fmi2_real_t value = std::get<double>(values);
      fmi2_import_set_real(parent_fmu, &valueReference, 1, &value);
      break;
    }
    case fmi2_base_type_int: {
      fmi2_integer_t value = std::get<int>(values);
      fmi2_import_set_integer(parent_fmu, &valueReference, 1, &value);
      break;
    }
    case fmi2_base_type_bool: {
      fmi2_boolean_t value = std::get<bool>(values) == true ? fmi2_true : fmi2_false;
      fmi2_import_set_boolean(parent_fmu, &valueReference, 1, &value);
      break;
    }
    default:
      break;
  }
}

}  // namespace fmi_adapter
