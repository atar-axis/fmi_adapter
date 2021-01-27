/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#include "fmi_adapter/FMIVariable.h"
#include <fmilib.h>



namespace fmi_adapter {


bool FMIBaseVariable::varInput_filter(std::shared_ptr<FMIBaseVariable> variable){
  return variable->getCausalityRaw() == fmi2_causality_enu_input;
}

bool FMIBaseVariable::varOutput_filter(std::shared_ptr<FMIBaseVariable> variable){
  return variable->getCausalityRaw() == fmi2_causality_enu_output;
}

bool FMIBaseVariable::varParam_filter(std::shared_ptr<FMIBaseVariable> variable){
  return variable->getCausalityRaw() == fmi2_causality_enu_parameter;
}

std::string FMIBaseVariable::rosifyName(const std::string& rawName) const
{
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

 std::string FMIBaseVariable::getNameRaw() const { return rawName; }
 fmi2_base_type_enu_t FMIBaseVariable::getTypeRaw() const { return rawType; }
 fmi2_causality_enu_t FMIBaseVariable::getCausalityRaw() const { return rawCausality; }
 std::string FMIBaseVariable::getNameRos() const { return rosifyName(rawName); }
 fmi2_value_reference_t FMIBaseVariable::getValueReference() const { return valueReference; }


FMIBaseVariable::FMIBaseVariable(fmi2_import_t* parent_fmu, fmi2_import_variable_t* element)
: parent(parent_fmu),
  valueReference(fmi2_import_get_variable_vr(element)),
  rawName(std::string(fmi2_import_get_variable_name(element))),
  rawType(fmi2_import_get_variable_base_type(element)),
  rawCausality(fmi2_import_get_causality(element))
{}

valueVariantTypes FMIBaseVariable::getValue()
{
  if (rawCausality != fmi2_causality_enu_output) {
    throw std::invalid_argument("Given variable is not an output variable!");
  }

  valueVariantTypes ret;

  switch(rawType){
    case fmi2_base_type_real:
    {
      fmi2_real_t value = 0.0;
      fmi2_import_get_real(parent, &valueReference, 1, &value);
      ret = (double) value;
      break;
    }
    case fmi2_base_type_int:
    {
      fmi2_integer_t value = 1;
      fmi2_import_get_integer(parent, &valueReference, 1, &value);
      ret = (int) value;
      break;
    }
    case fmi2_base_type_bool:
    {
      fmi2_boolean_t value;
      fmi2_import_get_boolean(parent, &valueReference, 1, &value);
      ret = (bool) (value == fmi2_true ? true : false);
      // !rm: ROS_INFO("returning %d", (bool) (value == fmi2_true ? true : false));
      break;
    }
    default:
      break;
  }

  return ret;
}

}
