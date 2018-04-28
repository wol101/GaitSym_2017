/*
 *  Contact.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/02/2002.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Contact_h
#define Contact_h

#include "NamedObject.h"

#include <ode/ode.h>

class SimulationWindow;

class Contact:public NamedObject
{
public:
    Contact();
    ~Contact();

    void SetJointID(dJointID jointID) { m_JointID = jointID; }

    dJointID GetJointID() { return m_JointID; }
    dJointFeedback* GetJointFeedback() { return &m_ContactJointFeedback; }
    dVector3* GetContactPosition() { return &m_ContactPosition; }

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
    void SetForceRadius(float forceSize) { m_ForceRadius = forceSize; }
    void SetForceScale(float forceScale) { m_ForceScale = forceScale; }
    float GetForceRadius() { return m_ForceRadius; }
    float GetForceScale() { return m_ForceScale; }
    void SetDrawContactForces(bool drawContactForces) { m_drawContactForces = drawContactForces; }
#endif

protected:

    dJointID m_JointID;
    dJointFeedback m_ContactJointFeedback;
    dVector3 m_ContactPosition;

#ifdef USE_QT
    float m_ForceRadius;
    float m_ForceScale;
    bool m_drawContactForces;
    double m_LastDrawTime;
#endif
};


#endif

