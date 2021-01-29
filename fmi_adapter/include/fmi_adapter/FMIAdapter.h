/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#pragma once

#include <fmilib.h>

namespace fmi_adapter {

// forward declarations (just include FMIAdapter.h instead of FMU.h and FMUVariable.h where needed)
class FMU;
class FMUVariable;

class FMIAdapter {
 public:
  FMIAdapter();
  ~FMIAdapter();

  // Copy and assignments not allowed
  FMIAdapter(const FMIAdapter& other) = delete;
  FMIAdapter& operator=(const FMIAdapter&) = delete;

  // add a new FMU
  void addSingleFMU(){};
  void config(){};
};

}  // namespace fmi_adapter