/*
 *  StepDriver.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 * Uses a pure stepped function to return a the value at a given time
 *
 */

#ifndef StepDriver_h
#define StepDriver_h

#include "Driver.h"

class StepDriver: public Driver
{
public:
    StepDriver();
    ~StepDriver();
    
    void SetValueDurationPairs(int size, double *valueDurationPairs);
    double GetValue(double time);
    
protected:
    double *m_ValueList;
    double *m_DurationList;
    int m_ListLength;
    int m_LastIndex;
};

#endif

