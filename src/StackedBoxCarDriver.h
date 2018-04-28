/*
 *  StackedBoxCarDriver.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Thu Feb 21 2013.
 *  Copyright (c) 2013 Bill Sellers. All rights reserved.
 *
 *  Uses a cyclic boxcar function to return a the value at a given time
 *
 */

#ifndef STACKEDBOXCARDRIVER_H
#define STACKEDBOXCARDRIVER_H

#include <vector>
#include "Driver.h"

class StackedBoxCarDriver : public Driver
{
public:
    StackedBoxCarDriver();
    ~StackedBoxCarDriver();

    void SetStackSize(int StackSize);
    void SetCycleTimes(double *CycleTimes);
    void SetDelays(double *Delays);
    void SetWidths(double *Widths);
    void SetHeights(double *Heights);

    std::vector<double> *GetCycleTimes() { return &m_CycleTimes; }
    std::vector<double> *GetDelays() { return &m_Delays; }
    std::vector<double> *GetWidths() { return &m_Widths; }
    std::vector<double> *GetHeights() { return &m_Heights; }

    double GetValue(double Time);

protected:
    std::vector<double> m_CycleTimes;
    std::vector<double> m_Delays;
    std::vector<double> m_Widths;
    std::vector<double> m_Heights;
    int m_StackSize;
};

#endif // STACKEDBOXCARDRIVER_H

