/*
 *  FixedDriver.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Jun 21 2014.
 *  Copyright (c) 2014 Bill Sellers. All rights reserved.
 *
 *  Outputs a fixed value
 *
 */

#include "FixedDriver.h"

#define RANGE(x, l, h) if (x < (l)) x = (l); if (x > (h)) x = (h);

FixedDriver::FixedDriver()
{
    mValue = 0;
}

FixedDriver::~FixedDriver()
{
}


double FixedDriver::GetValue(double time)
{
    if (time == m_LastTime) return m_LastValue;
    m_LastTime = time;

    double v = mValue;
    RANGE(v, m_MinValue, m_MaxValue);
    m_LastValue = v;
    return v;
}

void FixedDriver::SetValue(double value)
{
    mValue = value;
}

void FixedDriver::MultiplyValue(double mod)
{
    mValue *= mod;
}

void FixedDriver::AddValue(double mod)
{
    mValue += mod;
}
