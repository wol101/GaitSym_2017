/*
 *  CyclicDriver.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Dec 06 2003.
 *  Copyright (c) 2003 Bill Sellers. All rights reserved.
 *
 *  Uses a cyclic stepped function to return a the value at a given time
 *
 */

#ifndef CyclicDriver_h
#define CyclicDriver_h

#include "Driver.h"

class CyclicDriver: public Driver
{
public:
    CyclicDriver();
    ~CyclicDriver();
    
    void SetValueDurationPairs(int size, double *valueDurationPairs);
    void SetPhaseDelay(double phaseDelay) { m_PhaseDelay = phaseDelay; }; // 0 to 1
    double GetValue(double time);
    double GetCycleTime();
    
protected:
        
    double *m_ValueList;
    double *m_DurationList;
    double m_PhaseDelay;
    int m_ListLength;
    int m_LastIndex;
};

#endif
