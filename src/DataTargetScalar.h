/*
 *  DataTargetScalar.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#ifndef DATATARGETSCALAR_H
#define DATATARGETSCALAR_H

#include "DataTarget.h"

class Body;

class DataTargetScalar: public DataTarget
{
public:
    DataTargetScalar();
    ~DataTargetScalar();

    enum DataType
    {
        XP,
        YP,
        ZP,
        Q0,
        Q1,
        Q2,
        Q3,
        XV,
        YV,
        ZV,
        XRV,
        YRV,
        ZRV,
        Angle,
        MetabolicEnergy,
        MechanicalEnergy,
        DriverError
    };

    void SetDataType(DataType dataType) { m_DataType = dataType; }
    DataType GetDataType() { return m_DataType; }

    virtual double GetError(double time);
    virtual double GetError(int index);

    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow * /* window */) {};
#endif

protected:

    DataType m_DataType;
};

#endif // DATATARGETSCALAR_H
