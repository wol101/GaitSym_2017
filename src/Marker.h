/*
 *  Marker.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 22/08/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#ifndef Marker_h
#define Marker_h

#include "NamedObject.h"
#include "PGDMath.h"

class SimulationWindow;

struct MarkerInertia
{
    double I11;
    double I22;
    double I33;
    double I12;
    double I13;
    double I23;
};

class Marker: public NamedObject
{
public:

    Marker();
    virtual ~Marker();

    void SetBody(dBodyID body) { mBody = body; }
    dBodyID GetBody() { return mBody; }

    // these functions set the geom position relative to its body
    void SetPosition (double x, double y, double z)
    {
        mPosition.x = x; mPosition.y = y; mPosition.z = z;
    }
    void SetQuaternion(double q0, double q1, double q2, double q3)
    {
        mQuaternion.n = q0;
        mQuaternion.v.x = q1; mQuaternion.v.y = q2; mQuaternion.v.z = q3;
    }
    void SetPosition (const char *buf);
    void SetQuaternion(const char *buf);

    pgd::Vector GetPosition() { return mPosition; }
    pgd::Quaternion GetQuaternion() { return mQuaternion; }
    pgd::Vector GetWorldPosition();
    pgd::Quaternion GetWorldQuaternion();

    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
    void SetRadius(double v) { mRadius = v; }
#endif

protected:

    dBodyID mBody;
    pgd::Vector mPosition;
    pgd::Quaternion mQuaternion;

#ifdef USE_QT
    double mRadius;
#endif

};


#endif
