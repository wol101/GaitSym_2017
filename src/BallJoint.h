/*
 *  BallJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/12/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifndef BallJoint_h
#define BallJoint_h

#include "Joint.h"

class BallJoint: public Joint
{
public:

    BallJoint(dWorldID worldID, int mode = dAMotorEuler);

    void SetBallAnchor (double x, double y, double z);
    void SetBallAnchor(const char *buf);

    void SetStops(double a0Low, double a0High, double a1Low, double a1High, double a2Low, double a2High);
    void SetAxes(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, int axisMode);
    void SetAngles();
    void SetEulerReferenceVectors(dVector3 reference1, dVector3 reference2);

    void GetBallAnchor(dVector3 result);
    void GetBallAnchor2(dVector3 result);
    void GetEulerReferenceVectors(dVector3 reference1, dVector3 reference2);


    virtual void Update();
    virtual void Dump();

    // overridden method
    void Attach(Body *body1, Body* body2);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    dJointID m_MotorJointID;
    dJointFeedback m_MotorJointFeedback;
    int m_Mode;

};



#endif
