/*
 *  Driver.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 * Virtual class that all drivers descend from
 */

#ifndef Driver_h
#define Driver_h

#include "NamedObject.h"

class Drivable;

class Driver: public NamedObject
{
public:
    Driver();
    virtual ~Driver() ;

    void SetTarget(Drivable *target);
    Drivable *GetTarget() { return m_Target; }
    void SetMinMax(double minV, double maxV) { m_MinValue = minV; m_MaxValue = maxV; }
    void SetInterp(bool interp) { m_Interp = interp; }

    virtual double GetValue(double time) = 0;

    virtual void Dump();

protected:

    Drivable *m_Target;
    double m_MinValue;
    double m_MaxValue;
    bool m_Interp;
    double m_LastTime;
    double m_LastValue;

};

#endif
