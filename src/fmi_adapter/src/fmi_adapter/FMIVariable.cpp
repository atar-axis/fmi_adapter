/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#include "fmi_adapter/FMIVariable.h"
#include <fmilib.h>


namespace fmi_adapter {

FMIVariable::FMIVariable(
  std::string rawName_,
  fmi2_base_type_enu_t rawType_,
  fmi2_causality_enu_t rawCausality_
) : rawName(rawName_),
    rawType(rawType_),
    rawCausality(rawCausality_)
{
}

FMIVariable::FMIVariable(
  fmi2_import_variable_t* element
) : rawName(std::string(fmi2_import_get_variable_name(element))),
    rawType(fmi2_import_get_variable_base_type(element)),
    rawCausality(fmi2_import_get_causality(element))
{
}


std::string FMIVariable::rosifyName(const std::string& rawName) const
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

std::string FMIVariable::getNameRaw() const { return rawName; }
fmi2_base_type_enu_t FMIVariable::getTypeRaw() const { return rawType; }
fmi2_causality_enu_t FMIVariable::getCausalityRaw() const { return rawCausality; }
std::string FMIVariable::getNameRos() const { return rosifyName(rawName); }



}