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

#include <ode/ode.h>
#include <cmath>

#include "StackedBoxCarDriver.h"

#define RANGE(x, l, h) if (x < (l)) x = (l); if (x > (h)) x = (h);

StackedBoxCarDriver::StackedBoxCarDriver()
{
    m_StackSize = 0;
}

StackedBoxCarDriver::~StackedBoxCarDriver()
{
}

void StackedBoxCarDriver::SetStackSize(int StackSize)
{
    m_StackSize = StackSize;
    m_CycleTimes.resize(m_StackSize);
    m_Delays.resize(m_StackSize);
    m_Widths.resize(m_StackSize);
    m_Heights.resize(m_StackSize);
}

// these parameters control the shape of the box car and when it occurs
// CycleTime - this is the period of the boxcar
// Delay     - value from 0 to 1 used to control the phase of the function
// Width     - value from 0 to 1 used to control the width of the function
// Height    - value when the box car is active (otherwise the output is zero)
// Note: Delay and Width values are subtracted from the floor value to guarantee a value from 0 to 1

void StackedBoxCarDriver::SetCycleTimes(double *CycleTimes)
{
    for (int i = 0; i < m_StackSize; i++) m_CycleTimes[i] = CycleTimes[i];
}

void StackedBoxCarDriver::SetDelays(double *Delays)
{
    for (int i = 0; i < m_StackSize; i++) m_Delays[i] = Delays[i] - floor(Delays[i]);
}

void StackedBoxCarDriver::SetWidths(double *Widths)
{
    for (int i = 0; i < m_StackSize; i++) m_Widths[i] = Widths[i] - floor(Widths[i]);
}

void StackedBoxCarDriver::SetHeights(double *Heights)
{
    for (int i = 0; i < m_StackSize; i++) m_Heights[i] = Heights[i];
}


double StackedBoxCarDriver::GetValue(double Time)
{
    if (Time == m_LastTime) return m_LastValue;
    m_LastTime = Time;

    double Output = 0;
    double NormalisedCycleTime;
    double OffTime;

    for (int i = 0; i < m_StackSize; i++)
    {
        // get a normalised cycle time (value from 0 to 1)
        NormalisedCycleTime = (Time / m_CycleTimes[i]) - floor(Time / m_CycleTimes[i]);

        OffTime = m_Delays[i] + m_Widths[i];
        if (OffTime < 1) // no wrap case
        {
            if (NormalisedCycleTime > m_Delays[i] && NormalisedCycleTime < OffTime) Output += m_Heights[i];
        }
        else // wrap case
        {
            if (NormalisedCycleTime < OffTime - 1 || NormalisedCycleTime > m_Delays[i]) Output += m_Heights[i];
        }
    }

    RANGE(Output, m_MinValue, m_MaxValue);
    m_LastValue = Output;
    return Output;
}

