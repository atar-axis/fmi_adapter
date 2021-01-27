/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#include <string>

#include <FMI2/fmi2_enums.h>
#include <FMI2/fmi2_functions.h>
#include <FMI2/fmi2_import_variable.h>


namespace fmi_adapter {

class FMIVariable {

  private:
    std::string rawName;
    fmi2_base_type_enu_t rawType;
    fmi2_causality_enu_t rawCausality;

	std::string rosifyName(const std::string& rawName) const;

  public:
		FMIVariable(std::string, fmi2_base_type_enu_t, fmi2_causality_enu_t);
		FMIVariable(fmi2_import_variable_t*);
		std::string getNameRaw() const;
		fmi2_base_type_enu_t getTypeRaw() const;
		fmi2_causality_enu_t getCausalityRaw() const;

		std::string getNameRos() const;

};

}