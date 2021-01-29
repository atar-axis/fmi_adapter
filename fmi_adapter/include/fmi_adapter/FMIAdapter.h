/*
 *   Copyright (c) 2021 Florian Dollinger - dollinger.florian@gmx.de
 *   All rights reserved.
 */

#pragma once

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

}