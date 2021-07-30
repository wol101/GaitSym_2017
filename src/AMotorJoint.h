/*
 *  AMotorJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 07/01/2011.
 *  Copyright 2011 Bill Sellers. All rights reserved.
 *
 */

#ifndef AMOTORJOINT_H
#define AMOTORJOINT_H

#include "Joint.h"

class AMotorJoint: public Joint
{
public:

    AMotorJoint(dWorldID worldID);

    void SetStops(double low, double high);
    void SetAxis(double x, double y, double z, int axisMode);
    void SetAxis(const char *buf);
    void SetTargetVelocity(double targetVelocity);
    void SetMaxTorque(double maximumTorque);
 #ifdef EXPERIMENTAL
    void SetDynamicFriction(double dynamicFrictionIntercept, double dynamicFrictionSlope);
#endif

    double GetAngle();
    double GetAngleRate();

    virtual void Update();
    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    void SetAngle();
#ifdef EXPERIMENTAL
    void SetDynamicFriction();

    double m_dynamicFrictionIntercept;
    double m_dynamicFrictionSlope;
    bool m_dynamicFrictionFlag;
#endif
};


#endif // AMOTORJOINT_H
