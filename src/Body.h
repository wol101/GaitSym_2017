/*
 *  Body.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// this class is a wrapper for the ODE body

#ifndef Body_h
#define Body_h

#ifdef USE_QT
#include "SimulationWindow.h"
#include "GLUtils.h"
#endif

#include "NamedObject.h"
#include "Simulation.h"
#include "PGDMath.h"

class FacetedObject;

enum LimitTestResult
{
    WithinLimits = 0,
    XPosError = 1,
    YPosError = 2,
    ZPosError = 3,
    XVelError = 4,
    YVelError = 5,
    ZVelError = 6,
    NumericalError = 7
};

enum DragControl
{
    NoDrag = 0,
    DragCoefficients = 1,
    DragCylinderX = 2,
    DragCylinderY = 3,
    DragCylinderZ = 4
};

class Body: public NamedObject
{
public:

    Body(dWorldID worldID);
    ~Body();

    void SetPosition(double x, double y, double z);
    void SetQuaternion(double q0, double q1, double q2, double q3);
    void SetPosition(const char *buf);
    void SetQuaternion(const char *buf);
    void SetLinearVelocity(double x, double y, double z);
    void SetAngularVelocity(double x, double y, double z);
    void SetLinearVelocity(const char *buf);
    void SetAngularVelocity(const char *buf);

    void SetMass(const dMass *mass);

    void SetPositionLowBound(double x, double y, double z) { m_PositionLowBound[0] = x; m_PositionLowBound[1] = y; m_PositionLowBound[2] = z; };
    void SetPositionHighBound(double x, double y, double z) { m_PositionHighBound[0] = x; m_PositionHighBound[1] = y; m_PositionHighBound[2] = z; };
    void SetLinearVelocityLowBound(double x, double y, double z) { m_LinearVelocityLowBound[0] = x; m_LinearVelocityLowBound[1] = y; m_LinearVelocityLowBound[2] = z; };
    void SetLinearVelocityHighBound(double x, double y, double z) { m_LinearVelocityHighBound[0] = x; m_LinearVelocityHighBound[1] = y; m_LinearVelocityHighBound[2] = z; };

    void SetLinearDamping(double scale);
    void SetAngularDamping(double scale);
    void SetLinearDampingThreshold(double threshold);
    void SetAngularDampingThreshold(double threshold);
    void SetMaxAngularSpeed(double max_speed);


    const double *GetPosition();
    const double *GetQuaternion();
    const double *GetRotation();
    const double *GetLinearVelocity();
    const double *GetAngularVelocity();
    void GetRelativePosition(Body *rel, pgd::Vector *pos);
    void GetRelativeQuaternion(Body *rel, pgd::Quaternion *quat);
    void GetRelativeRotation(Body *rel, pgd::Matrix3x3 *rot);
    void GetRelativeLinearVelocity(Body *rel, pgd::Vector *vel);
    void GetRelativeAngularVelocity(Body *rel, pgd::Vector *rVel);
    double GetMass();
    double GetLinearKineticEnergy();
    void GetLinearKineticEnergy(dVector3 ke);
    double GetRotationalKineticEnergy();
    double GetGravitationalPotentialEnergy();

    dBodyID GetBodyID() { return m_BodyID; };

    LimitTestResult TestLimits();
    int SanityCheck(Body *otherBody, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight);


    // Utility
    void ParallelAxis(dMass *massProperties, const double *translation, const double *quaternion, dMass *newMassProperties);
    static void ParallelAxis(double x, double y, double z, // transformation from centre of mass to new location (m)
                             double mass, // mass (kg)
                             double ixx, double iyy, double izz, double ixy, double iyz, double izx, // moments of inertia kgm2
                             double ang, // rotation angle (radians)
                             double ax, double ay, double az, // axis of rotation - must be unit length
                             double *ixxp, double *iyyp, double *izzp, double *ixyp, double *iyzp, double *izxp); // transformed moments of inertia about new coordinate system

    // these values are really only used for graphics
    void SetOffset(double x, double y, double z) {m_Offset[0] = x; m_Offset[1] = y; m_Offset[2] = z; };
    const double *GetOffset() { return m_Offset; };

    virtual void Dump();
    virtual void WriteToXMLStream(std::ostream &outputStream);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    dWorldID m_WorldID;
    dBodyID m_BodyID;

    dVector3 m_PositionLowBound;
    dVector3 m_PositionHighBound;
    dVector3 m_LinearVelocityLowBound;
    dVector3 m_LinearVelocityHighBound;

    dVector3 m_Offset;


};

#endif
