/*
 *  CyclicDriver.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Dec 06 2003.
 *  Copyright (c) 2003 Bill Sellers. All rights reserved.
 *
 *  Uses a cyclic stepped function to return a the value at a given time
 *
 */

#include <iostream>

#include <ode/ode.h>

#include "CyclicDriver.h"
#include "Util.h"

CyclicDriver::CyclicDriver()
{
    m_ValueList = 0;
    m_DurationList = 0;
    m_ListLength = -1;
    m_LastIndex = 0;
    m_PhaseDelay = 0;
}

CyclicDriver::~CyclicDriver()
{
    if (m_DurationList) delete [] m_DurationList;
    if (m_ValueList) delete [] m_ValueList;
}

// Note list is delt0, v0, delt1, v1, delt2, v2 etc
// times are intervals not absolute simulation times
void CyclicDriver::SetValueDurationPairs(int size, double *valueDurationPairs)
{
    int i;
    if (size <= 0)
    {
        std::cerr << "CyclicDriver::SetValueDurationPairs error: size = " << size << "\n";
        return;
    }
    if (m_ListLength != size / 2)
    {
        if (m_DurationList) delete [] m_DurationList;
        if (m_ValueList) delete [] m_ValueList;
        m_ListLength = size / 2;
        m_DurationList = new double[m_ListLength + 1]; // this is +1 because the first entry is zero
        m_ValueList = new double[m_ListLength + 1]; // this lets the interp version wrap properly
    }
    m_DurationList[0] = 0;
    for (i = 0 ; i < m_ListLength; i++)
    {
        m_DurationList[i + 1] = valueDurationPairs[i * 2] + m_DurationList[i]; // fill the list with absolute times
        m_ValueList[i] = valueDurationPairs[i * 2 + 1];
    }
    m_ValueList[m_ListLength] = m_ValueList[0]; // this lets the interp version wrap properly
}

double CyclicDriver::GetValue(double time)
{
    if (time == m_LastTime) return m_LastValue;
    m_LastTime = time;

    double v;
    // account for phase
    // m_PhaseDelay is a relative value (0 to 1) but the time offset needs to be positive
    double timeOffset = m_DurationList[m_ListLength] - m_DurationList[m_ListLength] * m_PhaseDelay;
    time = time + timeOffset;

    double rem = fmod(time, m_DurationList[m_ListLength]);

    if (m_Interp == false)
    {
        // optimisation because most of the time this is called it just returns the value
        // used previously
        if (m_DurationList[m_LastIndex] <= rem && m_DurationList[m_LastIndex + 1] > rem)
        {
            v = m_ValueList[m_LastIndex];
        }
        else
        {
            m_LastIndex = Util::BinarySearchRange<double>(m_DurationList, m_ListLength, rem);

            if (m_LastIndex == -1) m_LastIndex = 0; // fixup for not found errors
            v = m_ValueList[m_LastIndex];
        }
    }
    else
    {
        // optimisation because most of the time this is called it just returns the value
        // used previously
        if (m_DurationList[m_LastIndex] <= rem && m_DurationList[m_LastIndex + 1] > rem)
        {
            v = ((rem - m_DurationList[m_LastIndex]) / (m_DurationList[m_LastIndex + 1] - m_DurationList[m_LastIndex])) *
                    (m_ValueList[m_LastIndex + 1] - m_ValueList[m_LastIndex]) + m_ValueList[m_LastIndex];
        }
        else
        {
            m_LastIndex = Util::BinarySearchRange<double>(m_DurationList, m_ListLength, rem);

            if (m_LastIndex == -1) m_LastIndex = 0; // fixup for not found errors
            v = ((rem - m_DurationList[m_LastIndex]) / (m_DurationList[m_LastIndex + 1] - m_DurationList[m_LastIndex])) *
                    (m_ValueList[m_LastIndex + 1] - m_ValueList[m_LastIndex]) + m_ValueList[m_LastIndex];
        }
    }


    if (v < m_MinValue) v = m_MinValue;
    if (v > m_MaxValue) v = m_MaxValue;
    m_LastValue = v;
    return v;
}

double CyclicDriver::GetCycleTime()
{
    return m_DurationList[m_ListLength];
}
