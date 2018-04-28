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

#ifndef BOXCARDRIVER_H
#define BOXCARDRIVER_H

#include "Driver.h"

class BoxCarDriver : public Driver
{
public:
    BoxCarDriver();
    ~BoxCarDriver();

    void SetBoxCarParameters(double CycleTime, double Delay, double Width, double Height);

    double GetCycleTime() { return m_CycleTime; }
    double GetDelay() { return m_Delay; }
    double GetWidth() { return m_Width; }
    double GetHeight() { return m_Height; }

    double GetValue(double Time);

protected:
    double m_CycleTime;
    double m_Delay;
    double m_Width;
    double m_Height;
};

#endif // BOXCARDRIVER_H

