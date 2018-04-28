/*
 *  Body.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// this class is a wrapper for the ODE body

#include <iostream>
#include <string>
#include <string.h>
#include <math.h>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include <ode/ode.h>

#include "Body.h"
#include "Simulation.h"
#include "PGDMath.h"
#include "Util.h"
#include "Marker.h"

#ifdef USE_QT
#include "GLUtils.h"
#include "FacetedObject.h"
#endif

// length of vector a
#define LENGTHOF(a) \
        sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define LENGTH2OF(a) \
        (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define OUTSIDERANGE(x, minX, maxX) \
        ((x) < (minX) || (x) > (maxX))

#define PSWAP(a,b,t) {t tmp; tmp=a; a=b; b=tmp;}

#define _I(i,j) I[(i)*4+(j)]

Body::Body(dWorldID worldID)
{
    m_BodyID = dBodyCreate (worldID);
    dBodySetData(m_BodyID, this);
    m_WorldID = worldID;

    for (int i = 0; i < 3; i++)
    {
        m_PositionLowBound[i] = -DBL_MAX;
        m_PositionHighBound[i] = DBL_MAX;
        m_LinearVelocityLowBound[i] = -DBL_MAX;
        m_LinearVelocityHighBound[i] = DBL_MAX;
    }

    SetOffset(0, 0, 0);


#ifdef USE_QT
    m_physRep = 0;
#endif
}

Body::~Body()
{
    dBodyDestroy (m_BodyID);
}

void Body::SetPosition(double x, double y, double z)
{
    dBodySetPosition(m_BodyID, x, y, z);
}

void Body::SetQuaternion(double q0, double q1, double q2, double q3)
{
    dQuaternion q;
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
    dBodySetQuaternion(m_BodyID, q);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
// bodyName x1 y1 z1 x2 y2 z2 - position such that x1,y1,z1 on bodyName has same world coordinates as x2,y2,z2 on local body
void Body::SetPosition(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (count == 1) // it must be a marker
    {
        Marker *marker = m_simulation->GetMarker(lBufPtrs[0]);
        if (marker == 0)
        {
            std::cerr << "Error in Body::SetPosition: Cannot find marker " << lBufPtrs[0] << "\n";
            return; // error condition
        }
        pgd::Vector wp = marker->GetWorldPosition();
        dBodySetPosition(m_BodyID, wp.x, wp.y, wp.z);
        return;
    }

    if (count == 2) // two marker definition
    {
        Marker *marker1 = m_simulation->GetMarker(lBufPtrs[0]);
        Marker *marker2 = m_simulation->GetMarker(lBufPtrs[1]);
        if (marker1 == 0 || marker2 == 0)
        {
            std::cerr << "Error in Body::SetPosition: Both markers must be defined " << lBufPtrs[0] << " " << lBufPtrs[1] << "\n";
            return; // error condition
        }
        // find the marker that is relative to this body and make it marker1
        if (marker2->GetBody() == m_BodyID) PSWAP(marker1, marker2, Marker *);
        if (marker2->GetBody() == m_BodyID)
        {
            std::cerr << "Error in Body::SetPosition: Only one marker of pair must be relative to the body \n";
            return; // error condition
        }
        if (marker1->GetBody() != m_BodyID)
        {
            std::cerr << "Error in Body::SetPosition: One marker of pair must be relative to the body \n";
            return; // error condition
        }

        pgd::Vector target = marker2->GetWorldPosition();
        pgd::Vector current = marker2->GetWorldPosition();
        pgd::Vector difference = target - current;
        dBodySetPosition(m_BodyID, difference.x, difference.y, difference.z);
        return;
    }

    if (count < 3)
    {
        std::cerr << "Error in Body::SetPosition\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodySetPosition(m_BodyID, pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Body::SetPosition\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodySetPosition(m_BodyID, pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in Body::SetPosition\n";
            return; // error condition
        }
    }
    if (count < 7)
    {
        dVector3 result;
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
        dBodyGetRelPointPos (theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
        dBodySetPosition(m_BodyID, result[0], result[1], result[2]);
    }
    else
    {
        // get world coordinates of x1,y1,z1
        dVector3 local1, world1, local2, world2;
        for (i = 0; i < 3; i++)
        {
            local1[i] = strtod(lBufPtrs[i + 1], 0);
            local2[i] = strtod(lBufPtrs[i + 4], 0);
        }
        dBodyGetRelPointPos (theBody->GetBodyID(), local1[0], local1[1], local1[2], world1);
        dBodyGetRelPointPos (m_BodyID, local2[0], local2[1], local2[2], world2);
        // add the error to the current position
        const double *p = dBodyGetPosition(m_BodyID);
        for (i = 0; i < 3; i++) pos[i] = p[i] + world1[i] - world2[i];
        dBodySetPosition(m_BodyID, pos[0], pos[1], pos[2]);

        // for checking
        // dBodyGetRelPointPos (m_BodyID, local2[0], local2[1], local2[2], world2);
        // for (i = 0; i < 3; i++) std::cerr << world1[i] << " " << world2[i] << "\n";
    }
}

// parses the quaternion allowing a relative position specified by BODY ID
// note quaternion is (qs,qx,qy,qz)
// s x y z - world coordinates
// bodyName s x y z - position relative to bodyName local coordinate system
void Body::SetQuaternion(const char *buf)
{
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (count == 1) // it must be a marker
    {
        Marker *marker = m_simulation->GetMarker(lBufPtrs[0]);
        if (marker == 0)
        {
            std::cerr << "Error in Body::SetQuaternion: Cannot find marker " << lBufPtrs[0] << "\n";
            return; // error condition
        }
        pgd::Quaternion wq = marker->GetWorldQuaternion();
        SetQuaternion(wq.n, wq.v.x, wq.v.y, wq.v.z);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Body::SetQuaternion\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        Util::GetQuaternion(&lBufPtrs[0], quaternion);
        dBodySetQuaternion(m_BodyID, quaternion);
        return;
    }

    if (count < 5)
    {
        std::cerr << "Error in Body::SetQuaternion\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            Util::GetQuaternion(&lBufPtrs[1], quaternion);
            dBodySetQuaternion(m_BodyID, quaternion);
            return;
        }
        else
        {
            std::cerr << "Error in Body::SetQuaternion\n";
            return; // error condition
        }
    }
    const double *q = theBody->GetQuaternion();
    pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
    Util::GetQuaternion(&lBufPtrs[1], quaternion);
    pgd::Quaternion qIn(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    pgd::Quaternion qNew = qBody * qIn;
    quaternion[0] = qNew.n;
    quaternion[1] = qNew.v.x;
    quaternion[2] = qNew.v.y;
    quaternion[3] = qNew.v.z;
    dBodySetQuaternion(m_BodyID, quaternion);
}

void Body::SetLinearVelocity(double x, double y, double z)
{
    dBodySetLinearVel(m_BodyID, x, y, z);
}

// parses the linear velocity allowing a relative velocity specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void Body::SetLinearVelocity(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 3)
    {
        std::cerr << "Error in Body::SetLinearVelocity\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetLinearVelocity(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Body::SetLinearVelocity\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetLinearVelocity(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in Body::SetLinearVelocity\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    const double *vRel = dBodyGetLinearVel(theBody->GetBodyID());
    SetLinearVelocity(result[0] + vRel[0], result[1] + vRel[1], result[2] + vRel[2]);
}

double Body::GetLinearKineticEnergy()
{
    // linear KE = 0.5 m v^2
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);

    const double *v = dBodyGetLinearVel(m_BodyID);
    double linearKE = 0.5 * mass.mass * LENGTH2OF(v);

    return linearKE;
}

void Body::GetLinearKineticEnergy(dVector3 ke)
{
    // linear KE = 0.5 m v^2
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);

    const double *v = dBodyGetLinearVel(m_BodyID);
    ke[0] =  0.5 * mass.mass * v[0] * v[0];
    ke[1] =  0.5 * mass.mass * v[1] * v[1];
    ke[2] =  0.5 * mass.mass * v[2] * v[2];

    return;
}

double Body::GetRotationalKineticEnergy()
{

    // rotational KE = 0.5 * o(t) * I * o
    // where o is rotational velocity vector and o(t) is the same but transposed

    dMass mass;
    dBodyGetMass(m_BodyID, &mass);

    const double *ow = dBodyGetAngularVel(m_BodyID);
    dVector3 o;
    dBodyVectorFromWorld (m_BodyID, ow[0], ow[1], ow[2], o);
    dVector3 o1;
    dMULTIPLY0_331(o1, mass.I, o);
    double rotationalKE = 0.5 * (o[0]*o1[0] + o[1]*o1[1] + o[2]*o1[2]);

    return rotationalKE;
}

double Body::GetGravitationalPotentialEnergy()
{
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);
    dVector3 g;
    dWorldGetGravity (m_WorldID, g);
    const double *p = dBodyGetPosition(m_BodyID);

    // gravitational PE = mgh
    double gravitationalPotentialEnergy = - mass.mass * (g[0]*p[0] + g[1]*p[1] + g[2]*p[2]);

    return gravitationalPotentialEnergy;
}

void Body::SetAngularVelocity(double x, double y, double z)
{
    dBodySetAngularVel(m_BodyID, x, y, z);
}

// parses the angular velocity allowing a relative angular velocity specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void Body::SetAngularVelocity(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 3)
    {
        std::cerr << "Error in Body::SetAngularVelocity";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetAngularVelocity(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Body::SetAngularVelocity";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetAngularVelocity(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in Body::SetAngularVelocity";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    const double *vARel = dBodyGetAngularVel(theBody->GetBodyID());
    SetAngularVelocity(result[0] + vARel[0], result[1] + vARel[1], result[2] + vARel[2]);
}

void Body::SetMass(const dMass *mass)
{
    dBodySetMass(m_BodyID, mass);
}

const double *Body::GetPosition()
{
    return dBodyGetPosition(m_BodyID);
}

const double *Body::GetQuaternion()
{
    return dBodyGetQuaternion(m_BodyID);
}

const double *Body::GetRotation()
{
    return dBodyGetRotation (m_BodyID);
}

const double *Body::GetLinearVelocity()
{
    return dBodyGetLinearVel(m_BodyID);
}

const double *Body::GetAngularVelocity()
{
    return dBodyGetAngularVel(m_BodyID);
}

void Body::GetRelativePosition(Body *rel, pgd::Vector *pos)
{
    dVector3 result;
    const double *p = GetPosition();
    if (rel)
    {
        dBodyGetPosRelPoint(rel->GetBodyID(), p[0], p[1], p[2], result);
        *pos = pgd::Vector(result[0], result[1], result[2]);
    }
    else
    {
        *pos = pgd::Vector(p[0], p[1], p[2]);
    }
}

void Body::GetRelativeQuaternion(Body *rel, pgd::Quaternion *quat)
{
    const double *qW = GetQuaternion();
    if (rel)
    {
        const double *qR = rel->GetQuaternion();
        pgd::Quaternion qWorld(qW[0], qW[1], qW[2], qW[3]);
        pgd::Quaternion qRelBody(qR[0], qR[1], qR[2], qR[3]);
        *quat = ~qRelBody * qWorld;
    }
    else
    {
        *quat = pgd::Quaternion(qW[0], qW[1], qW[2], qW[3]);
    }
}

void Body::GetRelativeRotation(Body *rel, pgd::Matrix3x3 *rot)
{
    const double *rW = GetRotation();
    if (rel)
    {
        const double *rR = rel->GetRotation();
        pgd::Matrix3x3 rWorld(rW[0], rW[1], rW[2], rW[4], rW[5], rW[6], rW[8], rW[9], rW[10]);
        pgd::Matrix3x3 rRelBody(rR[0], rR[1], rR[2], rR[4], rR[5], rR[6], rR[8], rR[9], rR[10]);
        *rot = rRelBody.Inverse() * rWorld;
    }
    else
    {
        *rot = pgd::Matrix3x3(rW[0], rW[1], rW[2], rW[4], rW[5], rW[6], rW[8], rW[9], rW[10]);
    }
}

void Body::GetRelativeLinearVelocity(Body *rel, pgd::Vector *vel)
{
    const double *v = GetLinearVelocity();
    if (rel)
    {
        const double *qR = rel->GetQuaternion();
        const double *vR = rel->GetLinearVelocity();
        pgd::Vector worldV(v[0], v[1], v[2]);
        pgd::Vector relV(vR[0], vR[1], vR[2]);
        pgd::Quaternion qRelBody(qR[0], qR[1], qR[2], qR[3]);
        *vel = QVRotate(~qRelBody, worldV - relV);
    }
    else
    {
        *vel = pgd::Vector(v[0], v[1], v[2]);
    }
}

void Body::GetRelativeAngularVelocity(Body *rel, pgd::Vector *rVel)
{
    const double *vR = GetAngularVelocity();
    if (rel)
    {
        const double *qR = rel->GetQuaternion();
        const double *vRR = rel->GetAngularVelocity();
        pgd::Vector worldVR(vR[0], vR[1], vR[2]);
        pgd::Vector relVR(vRR[0], vRR[1], vRR[2]);
        pgd::Quaternion qRelBody(qR[0], qR[1], qR[2], qR[3]);
        *rVel = QVRotate(~qRelBody, worldVR - relVR);
    }
    else
    {
        *rVel = pgd::Vector(vR[0], vR[1], vR[2]);
    }
}

double Body::GetMass()
{
    dMass mass;
    dBodyGetMass(m_BodyID, &mass);
    return mass.mass;
}


LimitTestResult Body::TestLimits()
{
    const double *p = dBodyGetPosition(m_BodyID);
    if (!std::isfinite(p[0])) return NumericalError;
    if (!std::isfinite(p[1])) return NumericalError;
    if (!std::isfinite(p[2])) return NumericalError;
    if (OUTSIDERANGE(p[0], m_PositionLowBound[0], m_PositionHighBound[0])) return XPosError;
    if (OUTSIDERANGE(p[1], m_PositionLowBound[1], m_PositionHighBound[1])) return YPosError;
    if (OUTSIDERANGE(p[2], m_PositionLowBound[2], m_PositionHighBound[2])) return ZPosError;

    const double *v = dBodyGetLinearVel(m_BodyID);
    if (!std::isfinite(v[0])) return NumericalError;
    if (!std::isfinite(v[1])) return NumericalError;
    if (!std::isfinite(v[2])) return NumericalError;
    if (OUTSIDERANGE(v[0], m_LinearVelocityLowBound[0], m_LinearVelocityHighBound[0])) return XVelError;
    if (OUTSIDERANGE(v[1], m_LinearVelocityLowBound[1], m_LinearVelocityHighBound[1])) return YVelError;
    if (OUTSIDERANGE(v[2], m_LinearVelocityLowBound[2], m_LinearVelocityHighBound[2])) return ZVelError;

    return WithinLimits;
}

void Body::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "NamedObject::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tXP\tYP\tZP\tXV\tYV\tZV\tQW\tQX\tQY\tQZ\tRVX\tRVY\tRVZ\tLKEX\tLKEY\tLKEZ\tRKE\tGPE\n";
        }
    }


    if (m_DumpStream)
    {
        const double *p = GetPosition();
        const double *v = GetLinearVelocity();
        const double *q = GetQuaternion();
        const double *rv = GetAngularVelocity();
        dVector3 ke;
        GetLinearKineticEnergy(ke);

        *m_DumpStream << m_simulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] <<
                "\t" << v[0] << "\t" << v[1] << "\t" << v[2] <<
                "\t" << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] <<
                "\t" << rv[0] << "\t" << rv[1] << "\t" << rv[2] <<
                "\t" << ke[0] << "\t" << ke[1] << "\t" << ke[2] <<
                "\t" << GetRotationalKineticEnergy() << "\t" << GetGravitationalPotentialEnergy() <<
                "\n";
    }
}

// a utility function to calculate moments of interia given an arbitrary translation and rotation
// assumes starting point is the moment of inertia at the centre of mass
#define _I(i,j) I[(i)*4+(j)]
void Body::ParallelAxis(dMass *massProperties, const double *translation, const double *quaternion, dMass *newMassProperties)
{
    double x, y, z; // transformation from centre of mass to new location (m)
    double mass; // mass (kg)
    double ixx,  iyy,  izz,  ixy,  iyz,  izx; // moments of inertia kgm2
    double ang; // rotation angle (radians)
    double ax, ay, az; // axis of rotation
    double ixxp, iyyp, izzp, ixyp, iyzp, izxp; // transformed moments of inertia about new coordinate system)

    x = translation[0];
    y = translation[1];
    z = translation[2];
    mass = massProperties->mass;
    ixx = massProperties->_I(0,0);
    iyy = massProperties->_I(1,1);
    izz = massProperties->_I(2,2);
    ixy = massProperties->_I(0,1);
    izx = massProperties->_I(0,2);
    iyz = massProperties->_I(1,2);

    ang = 2*acos(quaternion[0]);
    double magnitude = sqrt(SQUARE(quaternion[1]) + SQUARE(quaternion[2]) + SQUARE(quaternion[3]));
    if (magnitude <= 1e-10)
    {
        std::cerr << "Vector magnitude too low in Body::ParallelAxis\n";
    }
    ax = quaternion[1] / magnitude;
    ay = quaternion[2] / magnitude;
    az = quaternion[3] / magnitude;

    ParallelAxis(x, y, z, mass, ixx, iyy, izz, ixy, iyz, izx, ang, ax, ay, az, &ixxp, &iyyp, &izzp, &ixyp, &iyzp, &izxp);

    dMassSetParameters (newMassProperties, mass, 0, 0, 0, ixxp, iyyp, izzp, ixyp, izxp, iyzp);
}

// a utility function to calculate moments of interia given an arbitrary translation and rotation
void Body::ParallelAxis(double x, double y, double z, // transformation from centre of mass to new location (m)
                        double mass, // mass (kg)
                        double ixx, double iyy, double izz, double ixy, double iyz, double izx, // moments of inertia kgm2
                        double ang, // rotation angle (radians)
                        double ax, double ay, double az, // axis of rotation - must be unit length
                        double *ixxp, double *iyyp, double *izzp, double *ixyp, double *iyzp, double *izxp) // transformed moments of inertia about new coordinate system
{
    double cosang = cos(ang);
    double sinang = sin(ang);

    *ixxp = -(mass*(-(y*y) - (z*z))) + ((ax*ax)*(1 - cosang) + cosang)*
            (ixx*((ax*ax)*(1 - cosang) + cosang) + izx*(ax*az*(1 - cosang) + ay*sinang) +
             ixy*(ax*ay*(1 - cosang) - az*sinang)) + (ax*ay*(1 - cosang) - az*sinang)*
            (ixy*((ax*ax)*(1 - cosang) + cosang) + iyz*(ax*az*(1 - cosang) + ay*sinang) +
             iyy*(ax*ay*(1 - cosang) - az*sinang)) + (ax*az*(1 - cosang) + ay*sinang)*
            (izx*((ax*ax)*(1 - cosang) + cosang) + izz*(ax*az*(1 - cosang) + ay*sinang) +
             iyz*(ax*ay*(1 - cosang) - az*sinang));

    *iyyp = -(mass*(-(x*x) - (z*z))) + (ax*ay*(1 - cosang) + az*sinang)*
            (ixy*((ay*ay)*(1 - cosang) + cosang) + izx*(ay*az*(1 - cosang) - ax*sinang) +
             ixx*(ax*ay*(1 - cosang) + az*sinang)) + ((ay*ay)*(1 - cosang) + cosang)*
            (iyy*((ay*ay)*(1 - cosang) + cosang) + iyz*(ay*az*(1 - cosang) - ax*sinang) +
             ixy*(ax*ay*(1 - cosang) + az*sinang)) + (ay*az*(1 - cosang) - ax*sinang)*
            (iyz*((ay*ay)*(1 - cosang) + cosang) + izz*(ay*az*(1 - cosang) - ax*sinang) +
             izx*(ax*ay*(1 - cosang) + az*sinang));

    *izzp = -(mass*(-(x*x) - (y*y))) + (ax*az*(1 - cosang) - ay*sinang)*
            (izx*((az*az)*(1 - cosang) + cosang) + ixy*(ay*az*(1 - cosang) + ax*sinang) +
             ixx*(ax*az*(1 - cosang) - ay*sinang)) + (ay*az*(1 - cosang) + ax*sinang)*
            (iyz*((az*az)*(1 - cosang) + cosang) + iyy*(ay*az*(1 - cosang) + ax*sinang) +
             ixy*(ax*az*(1 - cosang) - ay*sinang)) + ((az*az)*(1 - cosang) + cosang)*
            (izz*((az*az)*(1 - cosang) + cosang) + iyz*(ay*az*(1 - cosang) + ax*sinang) +
             izx*(ax*az*(1 - cosang) - ay*sinang));

    *ixyp = -(mass*x*y) + (ax*ay*(1 - cosang) + az*sinang)*
            (ixx*((ax*ax)*(1 - cosang) + cosang) + izx*(ax*az*(1 - cosang) + ay*sinang) +
             ixy*(ax*ay*(1 - cosang) - az*sinang)) + ((ay*ay)*(1 - cosang) + cosang)*
            (ixy*((ax*ax)*(1 - cosang) + cosang) + iyz*(ax*az*(1 - cosang) + ay*sinang) +
             iyy*(ax*ay*(1 - cosang) - az*sinang)) + (ay*az*(1 - cosang) - ax*sinang)*
            (izx*((ax*ax)*(1 - cosang) + cosang) + izz*(ax*az*(1 - cosang) + ay*sinang) +
             iyz*(ax*ay*(1 - cosang) - az*sinang));

    *iyzp = -(mass*y*z) + (ax*az*(1 - cosang) - ay*sinang)*
            (ixy*((ay*ay)*(1 - cosang) + cosang) + izx*(ay*az*(1 - cosang) - ax*sinang) +
             ixx*(ax*ay*(1 - cosang) + az*sinang)) + (ay*az*(1 - cosang) + ax*sinang)*
            (iyy*((ay*ay)*(1 - cosang) + cosang) + iyz*(ay*az*(1 - cosang) - ax*sinang) +
             ixy*(ax*ay*(1 - cosang) + az*sinang)) + ((az*az)*(1 - cosang) + cosang)*
            (iyz*((ay*ay)*(1 - cosang) + cosang) + izz*(ay*az*(1 - cosang) - ax*sinang) +
             izx*(ax*ay*(1 - cosang) + az*sinang));

    *izxp = -(mass*x*z) + (ax*az*(1 - cosang) - ay*sinang)*
            (ixx*((ax*ax)*(1 - cosang) + cosang) + izx*(ax*az*(1 - cosang) + ay*sinang) +
             ixy*(ax*ay*(1 - cosang) - az*sinang)) + (ay*az*(1 - cosang) + ax*sinang)*
            (ixy*((ax*ax)*(1 - cosang) + cosang) + iyz*(ax*az*(1 - cosang) + ay*sinang) +
             iyy*(ax*ay*(1 - cosang) - az*sinang)) + ((az*az)*(1 - cosang) + cosang)*
            (izx*((ax*ax)*(1 - cosang) + cosang) + izz*(ax*az*(1 - cosang) + ay*sinang) +
             iyz*(ax*ay*(1 - cosang) - az*sinang));
}

// returns zero if position values are simply mirror images of each other
// also checks mass properties
int Body::SanityCheck(Body *otherBody, AxisType axis, const std::string & /*sanityCheckLeft */, const std::string & /* sanityCheckRight */)
{
    const double epsilon = 1e-10;
    const double *p1 = this->GetPosition();
    const double *p2 = otherBody->GetPosition();

    switch (axis)
    {
    case XAxis:
        if (fabs(p1[0] + p2[0]) > epsilon) return __LINE__;
        if (fabs(p1[1] - p2[1]) > epsilon) return __LINE__;
        if (fabs(p1[2] - p2[2]) > epsilon) return __LINE__;
        break;

    case YAxis:
        if (fabs(p1[0] - p2[0]) > epsilon) return __LINE__;
        if (fabs(p1[1] + p2[1]) > epsilon) return __LINE__;
        if (fabs(p1[2] - p2[2]) > epsilon) return __LINE__;
        break;

    case ZAxis:
        if (fabs(p1[0] - p2[0]) > epsilon) return __LINE__;
        if (fabs(p1[1] - p2[1]) > epsilon) return __LINE__;
        if (fabs(p1[2] + p2[2]) > epsilon) return __LINE__;
        break;
    }

    int i;
    dMass mass1, mass2;
    dBodyGetMass(m_BodyID, &mass1);
    dBodyGetMass(otherBody->GetBodyID(), &mass2);

    if (fabs(mass1.mass - mass2.mass) > epsilon) return __LINE__;
    for (i=0; i<3; i++) if (fabs(mass1.I[i] - mass2.I[i]) > epsilon) return __LINE__;
    for (i=4; i<7; i++) if (fabs(mass1.I[i] - mass2.I[i]) > epsilon) return __LINE__;
    for (i=8; i<11; i++) if (fabs(mass1.I[i] - mass2.I[i]) > epsilon) return __LINE__;

    return 0;
}

void Body::SetLinearDamping(double scale)
{
    dBodySetLinearDamping(m_BodyID, scale);
}

void Body::SetAngularDamping(double scale)
{
    dBodySetAngularDamping(m_BodyID, scale);
}

void Body::SetLinearDampingThreshold(double threshold)
{
    dBodySetLinearDampingThreshold(m_BodyID, threshold);
}

void Body::SetAngularDampingThreshold(double threshold)
{
    dBodySetAngularDampingThreshold(m_BodyID, threshold);
}

void Body::SetMaxAngularSpeed(double max_speed)
{
    dBodySetMaxAngularSpeed(m_BodyID, max_speed);
}


#ifdef USE_QT

void Body::Draw(SimulationWindow *window)
{
    if (m_physRep)
    {
        if (m_physRep->simulationWindow() == 0) // this must be the first time through
        {
            m_physRep->setSimulationWindow(window);
        }
    }
    m_physRep->SetVisible(m_Visible);

    // get into body local coordinates
    const double *pos = dBodyGetPosition(m_BodyID);
    const double *R = dBodyGetRotation(m_BodyID);
    // now do the drawing in local coordinates
     m_physRep->SetColour(m_Colour);
     m_physRep->SetDisplayRotation(R);
     m_physRep->SetDisplayPosition(pos[0], pos[1], pos[2]);
     m_physRep->Draw();

/*
    // from DrawStuff.cpp setTransform
    GLfloat matrix[16];
    matrix[0]=R[0];
    matrix[1]=R[4];
    matrix[2]=R[8];
    matrix[3]=0;
    matrix[4]=R[1];
    matrix[5]=R[5];
    matrix[6]=R[9];
    matrix[7]=0;
    matrix[8]=R[2];
    matrix[9]=R[6];
    matrix[10]=R[10];
    matrix[11]=0;
    matrix[12]=pos[0];
    matrix[13]=pos[1];
    matrix[14]=pos[2];
    matrix[15]=1;
    glPushMatrix();
    glMultMatrixf (matrix);

    // draw axes at origin (CM)
    if (gAxisFlag) GLUtils::DrawAxes(m_AxisSize[0], m_AxisSize[1], m_AxisSize[2]);
    glPopMatrix();
*/

}

#endif

void Body::WriteToXMLStream(std::ostream &outputStream)
{
    outputStream << "<BODY";
    outputStream << " ID=\"" << m_Name << "\"";
    const dReal *q = dBodyGetQuaternion(m_BodyID);
    outputStream << " Quaternion=\"World " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\""; // note quaternion is (qs,qx,qy,qz)
    const dReal *p = dBodyGetPosition(m_BodyID);
    outputStream << " Position=\"World " << p[0] << " " << p[1] << " " << p[2] << "\"";
    const dReal *v = dBodyGetLinearVel(m_BodyID);
    outputStream << " Position=\"World " << v[0] << " " << v[1] << " " << v[2] << "\"";
    const dReal *a = dBodyGetAngularVel(m_BodyID);
    outputStream << " Position=\"World " << a[0] << " " << a[1] << " " << a[2] << "\"";
    dMass mass;
    dBodyGetMass (m_BodyID, &mass);
    outputStream << " Mass=\"" << mass.mass << "\"";
    double I11 = mass._I(0,0);
    double I22 = mass._I(1,1);
    double I33 = mass._I(2,2);
    double I12 = mass._I(0,1);
    double I13 = mass._I(0,2);
    double I23 = mass._I(1,2);
    outputStream << " MOI=\"" << I11 << " " << I22 << " " << I33 << " " << I12 << " " << I13 << " " << I23 << "\"";
    outputStream << " PositionLowBound=\"" << m_PositionLowBound[0] << " " << m_PositionLowBound[1] << " " << m_PositionLowBound[2] << "\"";
    outputStream << " PositionHighBound=\"" << m_PositionHighBound[0] << " " << m_PositionHighBound[1] << " " << m_PositionHighBound[2] << "\"";
    outputStream << " LinearVelocityLowBound=\"" << m_LinearVelocityLowBound[0] << " " << m_LinearVelocityLowBound[1] << " " << m_LinearVelocityLowBound[2] << "\"";
    outputStream << " LinearVelocityHighBound=\"" << m_LinearVelocityHighBound[0] << " " << m_LinearVelocityHighBound[1] << " " << m_LinearVelocityHighBound[2] << "\"";
    outputStream << " LinearDamping=\"" << dBodyGetLinearDamping(m_BodyID) << "\"";
    outputStream << " AngularDamping=\"" << dBodyGetAngularDamping(m_BodyID) << "\"";
    outputStream << " LinearDampingThreshold=\"" << dBodyGetLinearDampingThreshold(m_BodyID) << "\"";
    outputStream << " AngularDampingThreshold=\"" << dBodyGetAngularDampingThreshold(m_BodyID) << "\"";
    outputStream << " MaxAngularSpeed=\"" << dBodyGetMaxAngularSpeed(m_BodyID) << "\"";


    /*
#ifdef USE_QT
    FacetedObject *facetedObject = new FacetedObject();

    // parameters that affect how the mesh is read in
    buf = DoXmlGetProp(cur, "VerticesAsSpheresRadius");
    if (buf)
    {
        facetedObject->SetVerticesAsSpheresRadius(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, "BadMesh");
    if (buf)
    {
        facetedObject->SetBadMesh(Util::Bool(buf));
    }

    THROWIFZERO(buf = DoXmlGetProp(cur, "GraphicFile"));
    std::string filename;
    if (m_GraphicsRoot.length() > 0) filename = std::string(m_GraphicsRoot) + std::string("/");
    filename += std::string((const char *)buf);
    facetedObject->ParseOBJFile(filename.c_str());

    // parameters for altering mesh after it has been read
    buf = DoXmlGetProp(cur, "Scale");
    if (buf)
    {
        int nTokens = DataFile::CountTokens(buf);
        if (nTokens == 1)
        {
            double scale = Util::Double(buf);
            if (scale != 1.0)
                facetedObject->Scale(scale, scale, scale);
        }
        else
        {
            if (nTokens == 3)
            {
                Util::Double(buf, 3, m_DoubleList);
                if (m_DoubleList[0] != 1.0 || m_DoubleList[1] != 1.0 || m_DoubleList[2] != 1.0)
                    facetedObject->Scale(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            }
            else
            {
                throw __LINE__;
            }
        }
    }
    double density = -1;
    buf = DoXmlGetProp(cur, "Density");
    if (buf)
    {
        density = Util::Double(buf);
    }
    if (density <= 0)
    {
        buf = DoXmlGetProp(cur, "Offset");
        if (buf)
        {
            Util::Double(buf, 3, m_DoubleList);
            theBody->SetOffset(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            facetedObject->Move(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
        }
    }

    bool clockwise = false;
    buf = DoXmlGetProp(cur, "Clockwise");
    if (buf)
    {
        clockwise = Util::Bool(buf);
    }
    // but we always want anticlockwise objects
    if (clockwise) facetedObject->ReverseWinding();

    facetedObject->SetColour(m_Interface.BodyColour);
    theBody->setPhysRep(facetedObject);
    theBody->SetColour(m_Interface.BodyColour);
    theBody->SetAxisSize(m_Interface.BodyAxisSize);

    if (density > 0)
    {
        // override the position values
        theBody->SetPosition(0, 0, 0);
        theBody->SetQuaternion(1, 0, 0, 0);

        facetedObject->CalculateMassProperties(&mass, density, false); // generally we assume anticlockwise winding
        std::cerr << *theBody->GetName() << " mass " << mass.mass
                << " CM " << mass.c[0] << " " << mass.c[1] << " " << mass.c[2] << " "
                << " I11_I22_I33 " << mass._I(0,0) << " " << mass._I(1,1) << " " << mass._I(2,2) << " "
                << " I12_I13_I23 " << mass._I(0,1) << " " << mass._I(0,2) << " " << mass._I(1,2) << "\n";
        const double *p = theBody->GetPosition();
        dVector3 newP;
        newP[0] = mass.c[0] + p[0]; newP[1] = mass.c[1] + p[1]; newP[2] = mass.c[2] + p[2];
        theBody->SetOffset(-mass.c[0], -mass.c[1], -mass.c[2]);
        facetedObject->Move(-mass.c[0], -mass.c[1], -mass.c[2]);
        facetedObject->CalculateMassProperties(&mass, density, false); // generally we assume anticlockwise winding
        std::cerr << *theBody->GetName() << " mass " << mass.mass
                << " CM " << mass.c[0] << " " << mass.c[1] << " " << mass.c[2] << " "
                << " I11_I22_I33 " << mass._I(0,0) << " " << mass._I(1,1) << " " << mass._I(2,2) << " "
                << " I12_I13_I23 " << mass._I(0,1) << " " << mass._I(0,2) << " " << mass._I(1,2) << "\n";
        mass.c[0] = mass.c[1] = mass.c[2]  = 0;
        theBody->SetMass(&mass);
        theBody->SetPosition(newP[0], newP[1], newP[2]);
    }

#endif
*/

    outputStream << "/>\n";
}

