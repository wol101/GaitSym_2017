/*
 *  Joint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// wrapper class for a joint

#ifndef Joint_h
#define Joint_h

#include "NamedObject.h"

#include <ode/ode.h>

class Body;
class SimulationWindow;

class Joint: public NamedObject
{
public:

    // Note constructor in derived functions need to set m_JointID
    // m_JointFeedback only needs to be set if GetFeedback is used (good idea though)

    Joint();
    virtual ~Joint();

    void Attach(Body *body1, Body *body2);
    Body *GetBody1() { return m_Body1; }
    Body *GetBody2() { return m_Body2; }

    dJointFeedback *GetFeedback();

    // some joints (particularly those with motors) need to do something before the simulation step
    virtual void Update() {};

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window) = 0;
#endif

protected:

    Body *m_Body1;
    Body *m_Body2;
    dJointID m_JointID;
    dJointFeedback m_JointFeedback;
};

#endif
