/*
 *  BoxCarDriver.h
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

#include "BoxCarDriver.h"

#define RANGE(x, l, h) if (x < (l)) x = (l); if (x > (h)) x = (h);

BoxCarDriver::BoxCarDriver()
{
    m_CycleTime = 1;
    m_Delay = 0;
    m_Width = 0;
    m_Height = 0;
}

BoxCarDriver::~BoxCarDriver()
{
}

// these parameters control the shape of the box car and when it occurs
// CycleTime - this is the period of the boxcar
// Delay     - value from 0 to 1 used to control the phase of the function
// Width     - value from 0 to 1 used to control the width of the function
// Height    - value when the box car is active (otherwise the output is zero)
// Note: Delay and Width values are subtracted from the floor value to guarantee a value from 0 to 1
void BoxCarDriver::SetBoxCarParameters(double CycleTime, double Delay, double Width, double Height)
{
    m_CycleTime = CycleTime;
    m_Delay = Delay - floor(Delay);
    m_Width = Width - floor(Width);
    m_Height = Height;
}

double BoxCarDriver::GetValue(double Time)
{
    if (Time == m_LastTime) return m_LastValue;
    m_LastTime = Time;

    // get a normalised cycle time (value from 0 to 1)
    double NormalisedCycleTime = (Time / m_CycleTime) - floor(Time / m_CycleTime);

    double Output = 0;
    double OffTime = m_Delay + m_Width;
    if (OffTime < 1) // no wrap case
    {
        if (NormalisedCycleTime > m_Delay && NormalisedCycleTime < OffTime) Output = m_Height;
    }
    else // wrap case
    {
        if (NormalisedCycleTime < OffTime - 1 || NormalisedCycleTime > m_Delay) Output = m_Height;
    }

    RANGE(Output, m_MinValue, m_MaxValue);
    m_LastValue = Output;
    return Output;
}
