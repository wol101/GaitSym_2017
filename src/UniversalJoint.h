/*
 *  UniversalJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/12/2010.
 *  Copyright 2010 Bill Sellers. All rights reserved.
 *
 */

#ifndef UniversalJoint_h
#define UniversalJoint_h

#include "Joint.h"

class UniversalJoint: public Joint
{
public:

    UniversalJoint(dWorldID worldID);

    void SetUniversalAnchor (double x, double y, double z);
    void SetUniversalAxis1(double x, double y, double z);
    void SetUniversalAxis2(double x, double y, double z);
    void SetUniversalAnchor (const char *buf);
    void SetUniversalAxis1(const char *buf);
    void SetUniversalAxis2(const char *buf);

    void SetStartAngleReference1(double startAngleReference);
    void SetStartAngleReference2(double startAngleReference);

    void GetUniversalAnchor(dVector3 result);
    void GetUniversalAnchor2(dVector3 result);
    void GetUniversalAxis1(dVector3 result);
    void GetUniversalAxis2(dVector3 result);

    double GetUniversalAngle1();
    double GetUniversalAngle1Rate();
    double GetUniversalAngle2();
    double GetUniversalAngle2Rate();

    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    double m_StartAngleReference1;
    double m_StartAngleReference2;

#ifdef USE_QT
    FacetedObject *m_physRep2;
#endif

};



#endif
