/*
 *  TwoCylinderWrapStrap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/12/2012.
 *  Copyright 2012 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include <cmath>
#include <string.h>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include "TwoCylinderWrapStrap.h"
#include "Body.h"
#include "PGDMath.h"
#include "DataFile.h"
#include "Simulation.h"
#include "Util.h"

#include "DebugControl.h"

#ifdef USE_QT
#include "FacetedPolyline.h"
#endif

TwoCylinderWrapStrap::TwoCylinderWrapStrap()
{
    PointForce *origin = new PointForce();
    PointForce *insertion = new PointForce();
    PointForce *cylinder1 = new PointForce();
    PointForce *cylinder2 = new PointForce();
    m_PointForceList.push_back(origin);
    m_PointForceList.push_back(insertion);
    m_PointForceList.push_back(cylinder1);
    m_PointForceList.push_back(cylinder2);

    m_Cylinder1Radius = 1;
    m_Cylinder2Radius = 1;
    m_WrapStatus = -1;
    m_NumWrapSegments = 16;
    m_NumPathCoordinates = 0;

#ifdef USE_QT
    m_PathCoordinates = new pgd::Vector[m_NumWrapSegments * 2 + 6];
#else
    m_PathCoordinates = 0;
#endif
}

TwoCylinderWrapStrap::~TwoCylinderWrapStrap()
{
    if (m_PathCoordinates) delete [] m_PathCoordinates;
}

void TwoCylinderWrapStrap::SetOrigin(Body *body, dVector3 point)
{
    m_OriginBody = body;
    m_PointForceList[0]->body = m_OriginBody;
    m_OriginPosition.x = point[0];
    m_OriginPosition.y = point[1];
    m_OriginPosition.z = point[2];
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void TwoCylinderWrapStrap::SetOrigin(Body *body, const char *buf)
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
        std::cerr << "Error in TwoCylinderWrapStrap::SetOrigin\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetOrigin(body, result);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in TwoCylinderWrapStrap::SetOrigin\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetOrigin(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in TwoCylinderWrapStrap::SetOrigin\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetOrigin(body, pos);
}

void TwoCylinderWrapStrap::SetInsertion(Body *body, dVector3 point)
{
    m_InsertionBody = body;
    m_PointForceList[1]->body = m_InsertionBody;
    m_InsertionPosition.x = point[0];
    m_InsertionPosition.y = point[1];
    m_InsertionPosition.z = point[2];
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void TwoCylinderWrapStrap::SetInsertion(Body *body, const char *buf)
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
        std::cerr << "Error in TwoCylinderWrapStrap::SetInsertion\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetInsertion(body, result);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in TwoCylinderWrapStrap::SetInsertion\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetInsertion(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in TwoCylinderWrapStrap::SetInsertion\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetInsertion(body, pos);
}

void TwoCylinderWrapStrap::SetCylinder1Body(Body *body)
{
    m_Cylinder1Body = body;
    m_PointForceList[2]->body = m_Cylinder1Body;
}

void TwoCylinderWrapStrap::SetCylinder1Position(double x, double y, double z)
{
    m_Cylinder1Position.x = x;
    m_Cylinder1Position.y = y;
    m_Cylinder1Position.z = z;
}

void TwoCylinderWrapStrap::GetCylinder1(Body **body, dVector3 position, double *radius, dQuaternion q)
{
    *body = m_Cylinder1Body;
    position[0] = m_Cylinder1Position.x;
    position[1] = m_Cylinder1Position.y;
    position[2] = m_Cylinder1Position.z;
    *radius = m_Cylinder1Radius;
    q[0] = m_CylinderQuaternion.n;
    q[1] = m_CylinderQuaternion.v.x;
    q[2] = m_CylinderQuaternion.v.y;
    q[3] = m_CylinderQuaternion.v.z;
}


// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void TwoCylinderWrapStrap::SetCylinder1Position(const char *buf)
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
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinder1Position\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(m_Cylinder1Body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetCylinder1Position(result[0], result[1], result[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinder1Position\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(m_Cylinder1Body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetCylinder1Position(result[0], result[1], result[2]);
            return;
        }
        else
        {
            std::cerr << "Error in TwoCylinderWrapStrap::SetCylinder1Position\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(m_Cylinder1Body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetCylinder1Position(pos[0], pos[1], pos[2]);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void TwoCylinderWrapStrap::SetCylinderAxis(const char *buf)
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
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinderAxis\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(m_Cylinder1Body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetCylinderAxis(result[0], result[1], result[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinderAxis\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyVectorFromWorld(m_Cylinder1Body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetCylinderAxis(result[0], result[1], result[2]);
            return;
        }
        else
        {
            std::cerr << "Error in TwoCylinderWrapStrap::SetCylinderAxis\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyVectorFromWorld(m_Cylinder1Body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetCylinderAxis(pos[0], pos[1], pos[2]);
}

void TwoCylinderWrapStrap::SetCylinderAxis(double x, double y, double z)
{
    pgd::Vector v2(x, y, z); // this is the target direction
    pgd::Vector v1(0, 0, 1); // and this is the Z axis we need to rotate

//    this is easy to explain but quite slow
//    // cross product will get us the rotation axis
//    pgd::Vector axis = v1 ^ v2;
//
//    // Use atan2 for a better angle.  If you use only cos or sin, you only get
//    // half the possible angles, and you can end up with rotations that flip around near
//    // the poles.
//
//    // cos angle obtained from dot product formula
//    // cos(a) = (s . e) / (||s|| ||e||)
//    double cosAng = v1 * v2; // (s . e)
//    double ls = v1.Magnitude();
//    ls = 1. / ls; // 1 / ||s||
//    double le = v2.Magnitude();
//    le = 1. / le; // 1 / ||e||
//    cosAng = cosAng * ls * le;
//
//    // sin angle obtained from cross product formula
//    // sin(a) = ||(s X e)|| / (||s|| ||e||)
//    double sinAng = axis.Magnitude(); // ||(s X e)||;
//    sinAng = sinAng * ls * le;
//    double angle = atan2(sinAng, cosAng); // rotations are in radians.
//
//    m_CylinderQuaternion = pgd::MakeQFromAxis(axis.x, axis.y, axis.z, angle);
    m_CylinderQuaternion = pgd::FindRotation(v1, v2);
}


void TwoCylinderWrapStrap::SetCylinderQuaternion(double q0, double q1, double q2, double q3)
{
    m_CylinderQuaternion.n = q0;
    m_CylinderQuaternion.v.x = q1;
    m_CylinderQuaternion.v.y = q2;
    m_CylinderQuaternion.v.z = q3;
    m_CylinderQuaternion.Normalize(); // this is the safest option
}

// parses the quaternion allowing a relative position specified by BODY ID
// note quaternion is (qs,qx,qy,qz)
// s x y z - world coordinates
// bodyName s x y z - position relative to bodyName local coordinate system
void TwoCylinderWrapStrap::SetCylinderQuaternion(const char *buf)
{
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 4)
    {
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinderQuaternion\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        const double *q = m_Cylinder1Body->GetQuaternion();
        pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
        Util::GetQuaternion(&lBufPtrs[0], quaternion);
        pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        pgd::Quaternion qLocal = ~qBody * qWorld;
        SetCylinderQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
        return;
    }

    if (count < 5)
    {
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinderQuaternion\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            const double *q = m_Cylinder1Body->GetQuaternion();
            pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
            Util::GetQuaternion(&lBufPtrs[1], quaternion);
            pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            pgd::Quaternion qLocal = ~qBody * qWorld;
            SetCylinderQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
            return;
        }
        else
        {
            std::cerr << "Error in TwoCylinderWrapStrap::SetCylinderQuaternion\n";
            return; // error condition
        }
    }

    // first get world quaternion
    const double *q2 = theBody->GetQuaternion();
    pgd::Quaternion qBody1(q2[0], q2[1], q2[2], q2[3]);
    Util::GetQuaternion(&lBufPtrs[1], quaternion);
    pgd::Quaternion qBody2(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    pgd::Quaternion qWorld = qBody1 * qBody2;

    // then set the local quaternion
    const double *q = m_Cylinder1Body->GetQuaternion();
    pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
    pgd::Quaternion qLocal = ~qBody * qWorld;
    SetCylinderQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);

}

void TwoCylinderWrapStrap::SetCylinder2Body(Body *body)
{
    m_Cylinder2Body = body;
    m_PointForceList[3]->body = m_Cylinder2Body;
}

void TwoCylinderWrapStrap::SetCylinder2Position(double x, double y, double z)
{
    m_Cylinder2Position.x = x;
    m_Cylinder2Position.y = y;
    m_Cylinder2Position.z = z;
}

void TwoCylinderWrapStrap::GetCylinder2(Body **body, dVector3 position, double *radius, dQuaternion q)
{
    *body = m_Cylinder2Body;
    position[0] = m_Cylinder2Position.x;
    position[1] = m_Cylinder2Position.y;
    position[2] = m_Cylinder2Position.z;
    *radius = m_Cylinder2Radius;
    q[0] = m_CylinderQuaternion.n;
    q[1] = m_CylinderQuaternion.v.x;
    q[2] = m_CylinderQuaternion.v.y;
    q[3] = m_CylinderQuaternion.v.z;
}


// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void TwoCylinderWrapStrap::SetCylinder2Position(const char *buf)
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
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinder2Position\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(m_Cylinder2Body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetCylinder2Position(result[0], result[1], result[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in TwoCylinderWrapStrap::SetCylinder2Position\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(m_Cylinder2Body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetCylinder2Position(result[0], result[1], result[2]);
            return;
        }
        else
        {
            std::cerr << "Error in TwoCylinderWrapStrap::SetCylinder2Position\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(m_Cylinder2Body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetCylinder2Position(pos[0], pos[1], pos[2]);
}


void TwoCylinderWrapStrap::Calculate(double deltaT)
{
    int i;
    // get the necessary body orientations and positions
    const double *q;
    q = dBodyGetQuaternion(m_OriginBody->GetBodyID());
    pgd::Quaternion qOriginBody(q[0], q[1], q[2], q[3]);
    q = dBodyGetPosition(m_OriginBody->GetBodyID());
    pgd::Vector vOriginBody(q[0], q[1], q[2]);
    q = dBodyGetQuaternion(m_InsertionBody->GetBodyID());
    pgd::Quaternion qInsertionBody(q[0], q[1], q[2], q[3]);
    q = dBodyGetPosition(m_InsertionBody->GetBodyID());
    pgd::Vector vInsertionBody(q[0], q[1], q[2]);
    q = dBodyGetQuaternion(m_Cylinder1Body->GetBodyID());
    pgd::Quaternion qCylinder1Body(q[0], q[1], q[2], q[3]);
    q = dBodyGetPosition(m_Cylinder1Body->GetBodyID());
    pgd::Vector vCylinder1Body(q[0], q[1], q[2]);
    q = dBodyGetQuaternion(m_Cylinder2Body->GetBodyID());
    pgd::Quaternion qCylinder2Body(q[0], q[1], q[2], q[3]);
    q = dBodyGetPosition(m_Cylinder2Body->GetBodyID());
    pgd::Vector vCylinder2Body(q[0], q[1], q[2]);

    // calculate some inverses
    pgd::Quaternion qCylinder1BodyInv = ~qCylinder1Body; // we only need qCylinder1Body because the m_CylinderQuaternion is relative to body 1
    pgd::Quaternion qCylinderQuaternionInv = ~m_CylinderQuaternion;

    // get the world coordinates of the origin and insertion
    pgd::Vector worldOriginPosition = QVRotate(qOriginBody, m_OriginPosition) + vOriginBody;
    pgd::Vector worldInsertionPosition = QVRotate(qInsertionBody, m_InsertionPosition) + vInsertionBody;

    // get the world coordinates of the cylinders
    pgd::Vector worldCylinder1Position = QVRotate(qCylinder1Body, m_Cylinder1Position) + vCylinder1Body;
    pgd::Vector worldCylinder2Position = QVRotate(qCylinder2Body, m_Cylinder2Position) + vCylinder2Body;

    // now rotate so the cylider axes are lined up on the z axis
    pgd::Vector cylinderOriginPosition = QVRotate(qCylinderQuaternionInv, QVRotate(qCylinder1BodyInv, worldOriginPosition));
    pgd::Vector cylinderInsertionPosition = QVRotate(qCylinderQuaternionInv, QVRotate(qCylinder1BodyInv, worldInsertionPosition));
    pgd::Vector cylinderCylinder1Position = QVRotate(qCylinderQuaternionInv, QVRotate(qCylinder1BodyInv, worldCylinder1Position));
    pgd::Vector cylinderCylinder2Position = QVRotate(qCylinderQuaternionInv, QVRotate(qCylinder1BodyInv, worldCylinder2Position));

    pgd::Vector theOriginForce;
    pgd::Vector theInsertionForce;
    pgd::Vector theCylinder1Force;
    pgd::Vector theCylinder1ForcePosition;
    pgd::Vector theCylinder2Force;
    pgd::Vector theCylinder2ForcePosition;
    m_LastLength = m_Length;
    double tension = 1; // normalised initially because tension is applied by muscle

    TwoCylinderWrap(cylinderOriginPosition, cylinderInsertionPosition, cylinderCylinder1Position, m_Cylinder1Radius,
                    cylinderCylinder2Position, m_Cylinder2Radius, tension, m_NumWrapSegments, M_PI,
                    theOriginForce, theInsertionForce, theCylinder1Force, theCylinder1ForcePosition,
                    theCylinder2Force, theCylinder2ForcePosition, &m_Length,
                    m_PathCoordinates, &m_NumPathCoordinates, &m_WrapStatus);

    // calculate the length and velocity
    if (deltaT != 0.0) m_Velocity = (m_Length - m_LastLength) / deltaT;
    else m_Velocity = 0;

    if (gDebug == TwoCylinderWrapStrapDebug)
    {
        *gDebugStream << "TwoCylinderWrapStrap::Calculate m_Name " << m_Name << " " <<
                         "cylinderOriginPosition " << cylinderOriginPosition.x << " " << cylinderOriginPosition.y << " " << cylinderOriginPosition.z << " " <<
                         "cylinderInsertionPosition " << cylinderInsertionPosition.x << " " << cylinderInsertionPosition.y << " " << cylinderInsertionPosition.z << "  " <<
                         "theOriginForce " << theOriginForce.x << " " << theOriginForce.y << " " << theOriginForce.z << " " <<
                         "theInsertionForce " << theInsertionForce.x << " " << theInsertionForce.y << " " << theInsertionForce.z << "  " <<
                         "theCylinder1ForcePosition " << theCylinder1ForcePosition.x << " " << theCylinder1ForcePosition.y << " " << theCylinder1ForcePosition.z << "  " <<
                         "theCylinder1Force " << theCylinder1Force.x << " " << theCylinder1Force.y << " " << theCylinder1Force.z << "  " <<
                         "theCylinder2ForcePosition " << theCylinder2ForcePosition.x << " " << theCylinder2ForcePosition.y << " " << theCylinder2ForcePosition.z << "  " <<
                         "theCylinder2Force " << theCylinder2Force.x << " " << theCylinder2Force.y << " " << theCylinder2Force.z << "  " <<
                         "m_Length " << m_Length << " m_Velocity " << m_Velocity << " m_WrapStatus " << m_WrapStatus << "\n";
    }
    // now rotate back to world reference frame

    theOriginForce = QVRotate(qCylinder1Body, QVRotate(m_CylinderQuaternion, theOriginForce));
    theInsertionForce = QVRotate(qCylinder1Body, QVRotate(m_CylinderQuaternion, theInsertionForce));
    theCylinder1Force = QVRotate(qCylinder1Body, QVRotate(m_CylinderQuaternion, theCylinder1Force));
    theCylinder1ForcePosition = QVRotate(qCylinder1Body, QVRotate(m_CylinderQuaternion, theCylinder1ForcePosition));
    theCylinder2Force = QVRotate(qCylinder1Body, QVRotate(m_CylinderQuaternion, theCylinder2Force));
    theCylinder2ForcePosition = QVRotate(qCylinder1Body, QVRotate(m_CylinderQuaternion, theCylinder2ForcePosition));


    PointForce *theOrigin = m_PointForceList[0];
    PointForce *theInsertion = m_PointForceList[1];
    PointForce *theCylinder1 = m_PointForceList[2];
    PointForce *theCylinder2 = m_PointForceList[3];
    theOrigin->vector[0] = theOriginForce.x; theOrigin->vector[1] = theOriginForce.y; theOrigin->vector[2] = theOriginForce.z;
    theOrigin->point[0] = worldOriginPosition.x; theOrigin->point[1] = worldOriginPosition.y; theOrigin->point[2] = worldOriginPosition.z;
    theInsertion->vector[0] = theInsertionForce.x; theInsertion->vector[1] = theInsertionForce.y; theInsertion->vector[2] = theInsertionForce.z;
    theInsertion->point[0] = worldInsertionPosition.x; theInsertion->point[1] = worldInsertionPosition.y; theInsertion->point[2] = worldInsertionPosition.z;
    theCylinder1->vector[0] = theCylinder1Force.x; theCylinder1->vector[1] = theCylinder1Force.y; theCylinder1->vector[2] = theCylinder1Force.z;
    theCylinder1->point[0] = theCylinder1ForcePosition.x; theCylinder1->point[1] = theCylinder1ForcePosition.y; theCylinder1->point[2] = theCylinder1ForcePosition.z;
    theCylinder2->vector[0] = theCylinder2Force.x; theCylinder2->vector[1] = theCylinder2Force.y; theCylinder2->vector[2] = theCylinder2Force.z;
    theCylinder2->point[0] = theCylinder2ForcePosition.x; theCylinder2->point[1] = theCylinder2ForcePosition.y; theCylinder2->point[2] = theCylinder2ForcePosition.z;

    // and handle the path coordinates
    for (i = 0; i < m_NumPathCoordinates; i++)
    {
        m_PathCoordinates[i] = QVRotate(qCylinder1Body, QVRotate(m_CylinderQuaternion, m_PathCoordinates[i]));
    }

    if (gDebug == StrapDebug)
    {
        pgd::Vector totalF(0, 0, 0);
        for (i = 0; i < (int)m_PointForceList.size(); i++)
        {
            *gDebugStream << "TwoCylinderWrapStrap::Calculate " <<
                    *m_PointForceList[i]->body->GetName() << " " <<
                    m_PointForceList[i]->point[0] << " " << m_PointForceList[i]->point[1] << " " << m_PointForceList[i]->point[2] << " " <<
                    m_PointForceList[i]->vector[0] << " " << m_PointForceList[i]->vector[1] << " " << m_PointForceList[i]->vector[2] << "\n";
            totalF.x += m_PointForceList[i]->vector[0]; totalF.y += m_PointForceList[i]->vector[1]; totalF.z += m_PointForceList[i]->vector[2];
        }
        *gDebugStream << "Total F " << totalF.x << " " << totalF.y << " " << totalF.z << "\n";
    }
}

// function to wrap a line around two parallel cylinders
// the cylinders are assumed to have their axes along the z axis and the wrapping is according
// to the right hand rule
// the coordinate system is right handed too

void TwoCylinderWrapStrap::TwoCylinderWrap(pgd::Vector &origin, pgd::Vector &insertion, pgd::Vector &cylinderPosition1, double radius1,
                                           pgd::Vector &cylinderPosition2, double radius2, double tension, int nPointsPerCylinderArc, double maxAngle,
                                           pgd::Vector &originForce, pgd::Vector &insertionForce, pgd::Vector &cylinderForce1, pgd::Vector &cylinderForcePosition1,
                                           pgd::Vector &cylinderForce2, pgd::Vector &cylinderForcePosition2, double *pathLength,
                                           pgd::Vector *pathCoordinates, int *numPathCoordinates, int *wrapOK)
{
    // this is the special case looking down the axis of the cylinder (i.e. xy plane)
    // this is standard tangent to a circle stuff

    // using letters as defined in TwoCylinderDiagram
    // set knowns to match diagram

    pgd::Vector O = origin;
    pgd::Vector I = insertion;
    pgd::Vector C = cylinderPosition1;
    pgd::Vector D = cylinderPosition2;
    double r = radius1;
    double s = radius2;

    const double small_angle = 1e-10;
    int number_of_tangents;

    *numPathCoordinates = 0;

    pgd::Vector E1, E2, H1, H2, G1, G2, F1, F2, J1, J2, K1, K2;

    // origin to first cylinder
    FindTangents(C, r, O, E1, E2, &number_of_tangents);
    if (number_of_tangents == 0)
    {
        *wrapOK = -1;
        return;
    }

    // insertion to second cylinder
    FindTangents(D, s, I, H1, H2, &number_of_tangents);
    if (number_of_tangents == 0)
    {
        *wrapOK = -1;
        return;
    }

    // now find line between cylinders
    pgd::Vector inner1_p1, inner1_p2, inner2_p1, inner2_p2; // not currently used
    FindCircleCircleTangents(C, r, D, s, F1, G1, F2, G2, inner1_p1, inner1_p2, inner2_p1, inner2_p2, &number_of_tangents);

    // now calculate the planar path length
    double cyl1_start_angle = atan2(E2.y - C.y, E2.x - C.x);
    double cyl1_end_angle = atan2(F2.y - C.y, F2.x - C.x);
    double cyl1_theta = cyl1_end_angle - cyl1_start_angle;
    if (cyl1_theta < 0) cyl1_theta = cyl1_theta + 2 * M_PI;
    double cyl2_start_angle = atan2(G2.y - D.y, G2.x - D.x);
    double cyl2_end_angle = atan2(H1.y - D.y, H1.x - D.x);
    double cyl2_theta = cyl2_end_angle - cyl2_start_angle;
    if (cyl2_theta < 0) cyl2_theta = cyl2_theta + 2 * M_PI;

    // use maxAngle to decide whether wrapping is appropriate
    // values between pi and 2pi are sensible

    double l1, l2, l3, l4, l5;
    double planar_path_length, delta_Z, cyl1_del_theta, cyl2_del_theta, theta;
    int i, j;

    if (cyl1_theta < maxAngle && cyl2_theta < maxAngle && cyl1_theta > small_angle && cyl2_theta > small_angle) // the double wrap case
    {

        *wrapOK = 1;

        l1 = vector_distance2d(O, E2);
        l2 = cyl1_theta * r;
        l3 = vector_distance2d(F2, G2);
        l4 = cyl2_theta * s;
        l5 = vector_distance2d(H1, I);

        planar_path_length = l1 + l2 + l3 + l4 + l5;
        delta_Z = I.z - O.z;

        E2.z = O.z + delta_Z * l1 / planar_path_length;
        F2.z = O.z + delta_Z * (l1 + l2) / planar_path_length;
        G2.z = O.z + delta_Z * (l1 + l2 + l3) / planar_path_length;
        H1.z = O.z + delta_Z * (l1 + l2 + l3 + l4) / planar_path_length;

        vector_with_magnitude(O, E2, tension, originForce);
        vector_with_magnitude(I, H1, tension, insertionForce);

        pgd::Vector betweenForce;
        vector_with_magnitude(F2, G2, tension, betweenForce);

        cylinderForce1.x = betweenForce.x - originForce.x;
        cylinderForce1.y = betweenForce.y - originForce.y;
        cylinderForce1.z = betweenForce.z - originForce.z;
        cylinderForcePosition1.x = cylinderPosition1.x;
        cylinderForcePosition1.y = cylinderPosition1.y;
        cylinderForcePosition1.z = (E2.z + F2.z) / 2;

        cylinderForce2.x = -betweenForce.x - insertionForce.x;
        cylinderForce2.y = -betweenForce.y - insertionForce.y;
        cylinderForce2.z = -betweenForce.z - insertionForce.z;
        cylinderForcePosition2.x = cylinderPosition2.x;
        cylinderForcePosition2.y = cylinderPosition2.y;
        cylinderForcePosition2.z = (G2.z + H1.z) / 2;

        *pathLength = sqrt(delta_Z * delta_Z + planar_path_length * planar_path_length);

        if (pathCoordinates)
        {
            i = 0;
            pathCoordinates[i] = O;
            i = i + 1;
            pathCoordinates[i] = E2;
            i = i + 1;

            // now fill in the missing bits of the 1st circle
            cyl1_del_theta = cyl1_theta / nPointsPerCylinderArc;
            theta = cyl1_start_angle;
            for (j = 1; j <= nPointsPerCylinderArc - 1; j++)
            {
                theta = theta + cyl1_del_theta;
                pathCoordinates[i].x = C.x + r * cos(theta);
                pathCoordinates[i].y = C.y + r * sin(theta);
                pathCoordinates[i].z = O.z + delta_Z * (l1 + ((double)j / (double)nPointsPerCylinderArc) * l2) / planar_path_length;
                i = i + 1;
            }

            pathCoordinates[i] = F2;
            i = i + 1;
            pathCoordinates[i] = G2;
            i = i + 1;

            // now fill in the missing bits of the 2nd circle
            cyl2_del_theta = cyl2_theta / nPointsPerCylinderArc;
            theta = cyl2_start_angle;
            for (j = 1; j <= nPointsPerCylinderArc - 1; j++)
            {
                theta = theta + cyl2_del_theta;
                pathCoordinates[i].x = D.x + s * cos(theta);
                pathCoordinates[i].y = D.y + s * sin(theta);
                pathCoordinates[i].z = O.z + delta_Z * (l1 + l2 + l3 + ((double)j / (double)nPointsPerCylinderArc) * l4) / planar_path_length;
                i = i + 1;
            }

            pathCoordinates[i] = H1;
            i = i + 1;
            pathCoordinates[i] = I;
            i = i + 1;
            *numPathCoordinates = i;
        }
        return;
    }

    if (cyl1_theta < maxAngle && cyl1_theta > small_angle) // try cyl 1 wrapping
    {
        // insertion to first cylinder
        FindTangents(C, r, I, K1, K2, &number_of_tangents);
        if (number_of_tangents == 0)
        {
            *wrapOK = -1;
            return;
        }

        // calculate new angle
        cyl1_start_angle = atan2(E2.y - C.y, E2.x - C.x);
        cyl1_end_angle = atan2(K1.y - C.y, K1.x - C.x);
        cyl1_theta = cyl1_end_angle - cyl1_start_angle;
        if (cyl1_theta < 0) cyl1_theta = cyl1_theta + 2 * M_PI;

        if (cyl1_theta < maxAngle)
        {
            *wrapOK = 2;

            l1 = vector_distance2d(O, E2);
            l2 = cyl1_theta * r;
            l3 = vector_distance2d(K1, I);

            planar_path_length = l1 + l2 + l3;
            delta_Z = I.z - O.z;

            E2.z = O.z + delta_Z * l1 / planar_path_length;
            K1.z = O.z + delta_Z * (l1 + l2) / planar_path_length;

             vector_with_magnitude(O, E2, tension, originForce);
             vector_with_magnitude(I, K1, tension, insertionForce);

            cylinderForce1.x = -insertionForce.x - originForce.x;
            cylinderForce1.y = -insertionForce.y - originForce.y;
            cylinderForce1.z = -insertionForce.z - originForce.z;
            cylinderForcePosition1.x = cylinderPosition1.x;
            cylinderForcePosition1.y = cylinderPosition1.y;
            cylinderForcePosition1.z = (E2.z + K1.z) / 2;

            cylinderForce2.x = 0;
            cylinderForce2.y = 0;
            cylinderForce2.z = 0;
            cylinderForcePosition2.x = 0;
            cylinderForcePosition2.y = 0;
            cylinderForcePosition2.z = 0;

            *pathLength = sqrt(delta_Z * delta_Z + planar_path_length * planar_path_length);

            if (pathCoordinates)
            {
                i = 0;
                pathCoordinates[i] = O;
                i = i + 1;
                pathCoordinates[i] = E2;
                i = i + 1;

                // now fill in the missing bits of the 1st circle
                cyl1_del_theta = cyl1_theta / nPointsPerCylinderArc;
                theta = cyl1_start_angle;
                for (j = 1; j <= nPointsPerCylinderArc - 1; j++)
                {
                    theta = theta + cyl1_del_theta;
                    pathCoordinates[i].x = C.x + r * cos(theta);
                    pathCoordinates[i].y = C.y + r * sin(theta);
                    pathCoordinates[i].z = O.z + delta_Z * (l1 + ((double)j / (double)nPointsPerCylinderArc) * l2) / planar_path_length;
                    i = i + 1;
                }

                pathCoordinates[i] = K1;
                i = i + 1;
                pathCoordinates[i] = I;
                i = i + 1;
                *numPathCoordinates = i;
            }

            return;
        }
    }

    if (cyl2_theta < maxAngle && cyl2_theta > small_angle) // try cyl 2 wrapping
    {
        // insertion to first cylinder
        FindTangents(D, s, O, J1, J2, &number_of_tangents);
        if (number_of_tangents == 0)
        {
            *wrapOK = -1;
            return;
        }

        // calculate new angle
        cyl2_start_angle = atan2(J2.y - D.y, J2.x - D.x);
        cyl2_end_angle = atan2(H1.y - D.y, H1.x - D.x);
        cyl2_theta = cyl2_end_angle - cyl2_start_angle;
        if (cyl2_theta < 0) cyl2_theta = cyl2_theta + 2 * M_PI;

        if (cyl2_theta < maxAngle)
        {
            *wrapOK = 3;

            l1 = vector_distance2d(O, J2);
            l2 = cyl2_theta * s;
            l3 = vector_distance2d(H1, I);

            planar_path_length = l1 + l2 + l3;
            delta_Z = I.z - O.z;

            J2.z = O.z + delta_Z * l1 / planar_path_length;
            H1.z = O.z + delta_Z * (l1 + l2) / planar_path_length;

            vector_with_magnitude(O, J2, tension, originForce);
            vector_with_magnitude(I, H1, tension, insertionForce);

            cylinderForce2.x = -insertionForce.x - originForce.x;
            cylinderForce2.y = -insertionForce.y - originForce.y;
            cylinderForce2.z = -insertionForce.z - originForce.z;
            cylinderForcePosition2.x = cylinderPosition2.x;
            cylinderForcePosition2.y = cylinderPosition2.y;
            cylinderForcePosition2.z = (J2.z + H1.z) / 2;

            cylinderForce1.x = 0;
            cylinderForce1.y = 0;
            cylinderForce1.z = 0;
            cylinderForcePosition1.x = 0;
            cylinderForcePosition1.y = 0;
            cylinderForcePosition1.z = 0;

            *pathLength = sqrt(delta_Z * delta_Z + planar_path_length * planar_path_length);

            if (pathCoordinates)
            {
                i = 0;
                pathCoordinates[i] = O;
                i = i + 1;
                pathCoordinates[i] = J2;
                i = i + 1;

                // now fill in the missing bits of the 1st circle
                cyl2_del_theta = cyl2_theta / nPointsPerCylinderArc;
                theta = cyl2_start_angle;
                for (j = 1; j <= nPointsPerCylinderArc - 1; j++)
                {
                    theta = theta + cyl2_del_theta;
                    pathCoordinates[i].x = D.x + s * cos(theta);
                    pathCoordinates[i].y = D.y + s * sin(theta);
                    pathCoordinates[i].z = O.z + delta_Z * (l1 + ((double)j / (double)nPointsPerCylinderArc) * l2) / planar_path_length;
                    i = i + 1;
                }

                pathCoordinates[i] = H1;
                i = i + 1;
                pathCoordinates[i] = I;
                i = i + 1;
                *numPathCoordinates = i;
            }

            return;
        }
    }


    // if we get here then no wrapping is possible

    *wrapOK = 0;

    *pathLength = vector_distance3d(O, I);

    vector_with_magnitude(O, I, tension, originForce);
    insertionForce.x = -originForce.x;
    insertionForce.y = -originForce.y;
    insertionForce.z = -originForce.z;

    cylinderForce1.x = 0;
    cylinderForce1.y = 0;
    cylinderForce1.z = 0;
    cylinderForcePosition1.x = 0;
    cylinderForcePosition1.y = 0;
    cylinderForcePosition1.z = 0;
    cylinderForce2.x = 0;
    cylinderForce2.y = 0;
    cylinderForce2.z = 0;
    cylinderForcePosition2.x = 0;
    cylinderForcePosition2.y = 0;
    cylinderForcePosition2.z = 0;

    if (pathCoordinates)
    {
        i = 0;
        pathCoordinates[i] = O;
        i = i + 1;
        pathCoordinates[i] = I;
        i = i + 1;
        *numPathCoordinates = i;
    }

    return;
}

// Adapted from http://www.vb-helper.com/howto_net_circle_circle_tangents.html
// Find the tangent points for these two circles.
// Return the number of tangents: 4, 2, or 0.
void TwoCylinderWrapStrap::FindCircleCircleTangents(pgd::Vector &c1, double radius1, pgd::Vector &c2, double radius2,
                                                    pgd::Vector &outer1_p1, pgd::Vector &outer1_p2, pgd::Vector &outer2_p1, pgd::Vector &outer2_p2,
                                                    pgd::Vector &inner1_p1, pgd::Vector &inner1_p2, pgd::Vector &inner2_p1, pgd::Vector &inner2_p2, int *number_of_tangents)
{

    //  Make sure radius1 <= radius2.
    if (radius1 > radius2)
    {
        // Call this method switching the circles.
        FindCircleCircleTangents(c2, radius2, c1, radius1, outer2_p2, outer2_p1, outer1_p2, outer1_p1, inner2_p2, inner2_p1, inner1_p2, inner1_p1, number_of_tangents);
        return;
    }

    // ***************************
    // * Find the outer tangents *
    // ***************************
    double radius2a = radius2 - radius1;
    FindTangents(c2, radius2a, c1, outer1_p2, outer2_p2, number_of_tangents);
    if (*number_of_tangents == 0)
        return; // There are no tangents.

    // Get the vector perpendicular to the
    // first tangent with length radius1.
    double v1x = -(outer1_p2.y - c1.y);
    double v1y = outer1_p2.x - c1.x;
    double v1_length = sqrt(v1x * v1x + v1y * v1y);
    v1x = v1x * radius1 / v1_length;
    v1y = v1y * radius1 / v1_length;
    // Offset the tangent vector's points.
    outer1_p1.x = c1.x + v1x;
    outer1_p1.y = c1.y + v1y;
    outer1_p2.x = outer1_p2.x + v1x;
    outer1_p2.y = outer1_p2.y + v1y;

    // Get the vector perpendicular to the
    // second tangent with length radius1.
    double v2x = outer2_p2.y - c1.y;
    double v2y = -(outer2_p2.x - c1.x);
    double v2_length = sqrt(v2x * v2x + v2y * v2y);
    v2x = v2x * radius1 / v2_length;
    v2y = v2y * radius1 / v2_length;
    // Offset the tangent vector's points.
    outer2_p1.x = c1.x + v2x;
    outer2_p1.y = c1.y + v2y;
    outer2_p2.x = outer2_p2.x + v2x;
    outer2_p2.y = outer2_p2.y + v2y;

    // If the circles intersect, then there are no inner
    // tangents.
    double dx = c2.x - c1.x;
    double dy = c2.y - c1.y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist <= radius1 + radius2)
    {
        *number_of_tangents = 2;
        return;
    }

    // ***************************
    // * Find the inner tangents *
    // ***************************
    double radius1a = radius1 + radius2;
    FindTangents(c1, radius1a, c2, inner1_p2, inner2_p2, number_of_tangents);

    // Get the vector perpendicular to the
    // first tangent with length radius2.
    v1x = inner1_p2.y - c2.y;
    v1y = -(inner1_p2.x - c2.x);
    v1_length = sqrt(v1x * v1x + v1y * v1y);
    v1x = v1x * radius2 / v1_length;
    v1y = v1y * radius2 / v1_length;
    // Offset the tangent vector's points.
    inner1_p1.x = c2.x + v1x;
    inner1_p1.y = c2.y + v1y;
    inner1_p2.x = inner1_p2.x + v1x;
    inner1_p2.y = inner1_p2.y + v1y;

    // Get the vector perpendicular to the
    // second tangent with length radius2.
    v2x = -(inner2_p2.y - c2.y);
    v2y = inner2_p2.x - c2.x;
    v2_length = sqrt(v2x * v2x + v2y * v2y);
    v2x = v2x * radius2 / v2_length;
    v2y = v2y * radius2 / v2_length;
    // Offset the tangent vector's points.
    inner2_p1.x = c2.x + v2x;
    inner2_p1.y = c2.y + v2y;
    inner2_p2.x = inner2_p2.x + v2x;
    inner2_p2.y = inner2_p2.y + v2y;

    *number_of_tangents = 4;
    return;
}


// Adapted from http://www.vb-helper.com/howto_net_find_circle_tangents.html
// Find the tangent points for this circle and external
// point.
// Return the number of tangents: 2, or 0.
void TwoCylinderWrapStrap::FindTangents(pgd::Vector &center, double radius, pgd::Vector &external_point, pgd::Vector &pt1, pgd::Vector &pt2, int *number_of_tangents)
{
    // Find the distance squared from the
    // external point to the circle's center.
    double dx = center.x - external_point.x;
    double dy = center.y - external_point.y;
    double D_squared = dx * dx + dy * dy;
    if (D_squared < radius * radius)
    {
        *number_of_tangents = 0;
        return;
    }

    // Find the distance from the external point
    // to the tangent points.
    double L = sqrt(D_squared - radius * radius);

    // Find the points of intersection between
    // the original circle and the circle with
    // center external_point and radius dist.

    int number_of_intersections;
    FindCircleCircleIntersections(center.x, center.y, radius, external_point.x, external_point.y, L, pt1, pt2, &number_of_intersections);
    *number_of_tangents = 2;
    return;
}

// Adapted from http://www.vb-helper.com/howto_net_circle_circle_intersection.html
// Find the points where the two circles intersect.
void TwoCylinderWrapStrap::FindCircleCircleIntersections(double cx0, double cy0, double radius0, double cx1, double cy1, double radius1,
                                                         pgd::Vector &intersection1, pgd::Vector &intersection2, int *number_of_intersections)
{
    // Find the distance between the centers.
    double dx = cx0 - cx1;
    double dy = cy0 - cy1;
    double dist = sqrt(dx * dx + dy * dy);

    // See how many solutions there are.
    if (dist > radius0 + radius1)
    {
        // No solutions, the circles are too far apart.
        *number_of_intersections = 0;
        return;
    }

    if (dist < fabs(radius0 - radius1))
    {
        // No solutions, one circle contains the other.
        *number_of_intersections = 0;
        return;
    }

    if ((dist == 0) && (radius0 == radius1)) // WARNING - shouldn't compare equality for FP numbers
    {
        // No solutions, the circles coincide.
        *number_of_intersections = 0;
        return;
    }

    // Find a and h.
    double a = (radius0 * radius0 - radius1 * radius1 + dist * dist) / (2 * dist);
    double h = sqrt(radius0 * radius0 - a * a);

    // Find P2.
    double cx2 = cx0 + a * (cx1 - cx0) / dist;
    double cy2 = cy0 + a * (cy1 - cy0) / dist;

    // Get the points P3.
    intersection1.x = cx2 + h * (cy1 - cy0) / dist;
    intersection1.y = cy2 - h * (cx1 - cx0) / dist;
    intersection2.x = cx2 - h * (cy1 - cy0) / dist;
    intersection2.y = cy2 + h * (cx1 - cx0) / dist;

    // See if we have 1 or 2 solutions.
    if (dist == radius0 + radius1) // WARNING - shouldn't compare equality for FP numbers
        *number_of_intersections = 1;
    else
        *number_of_intersections = 2;

    return;
}

// calculate the 2D length of a vector
double TwoCylinderWrapStrap::vector_distance2d(pgd::Vector &v1, pgd::Vector v2)
{
    return sqrt((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y));
}

// calculate the 3D length of a vector
double TwoCylinderWrapStrap::vector_distance3d(pgd::Vector &v1, pgd::Vector &v2)
{
    return sqrt((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y) + (v2.z - v1.z) * (v2.z - v1.z));
}

// return the vector in the direction of v1 to v2 with magnitude specified
void TwoCylinderWrapStrap::vector_with_magnitude(pgd::Vector &v1, pgd::Vector &v2, double magnitude, pgd::Vector &v)
{
    pgd::Vector del_v = v2 - v1;
    double mag = vector_distance3d(v1, v2);
    v.x = magnitude * del_v.x / mag;
    v.y = magnitude * del_v.y / mag;
    v.z = magnitude * del_v.z / mag;
    return;
}

int TwoCylinderWrapStrap::SanityCheck(Strap *otherStrap, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight)
{
    const double epsilon = 1e-10;

    TwoCylinderWrapStrap *other = dynamic_cast<TwoCylinderWrapStrap *>(otherStrap);
    if (other == 0) return __LINE__;

    if (fabs(this->m_Cylinder1Radius - other->m_Cylinder1Radius) > epsilon) return __LINE__;
    if (fabs(this->m_Cylinder2Radius - other->m_Cylinder2Radius) > epsilon) return __LINE__;

    // first check attachment errors
    switch (axis)
    {
    case XAxis:
        if (fabs(this->m_OriginPosition.x + other->m_OriginPosition.x) > epsilon) return __LINE__;
        if (fabs(this->m_OriginPosition.y - other->m_OriginPosition.y) > epsilon) return __LINE__;
        if (fabs(this->m_OriginPosition.z - other->m_OriginPosition.z) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.x + other->m_InsertionPosition.x) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.y - other->m_InsertionPosition.y) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.z - other->m_InsertionPosition.z) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.x + other->m_Cylinder1Position.x) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.y - other->m_Cylinder1Position.y) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.z - other->m_Cylinder1Position.z) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.x + other->m_Cylinder2Position.x) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.y - other->m_Cylinder2Position.y) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.z - other->m_Cylinder2Position.z) > epsilon) return __LINE__;
        break;

    case YAxis:
        if (fabs(this->m_OriginPosition.x - other->m_OriginPosition.x) > epsilon) return __LINE__;
        if (fabs(this->m_OriginPosition.y + other->m_OriginPosition.y) > epsilon) return __LINE__;
        if (fabs(this->m_OriginPosition.z - other->m_OriginPosition.z) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.x - other->m_InsertionPosition.x) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.y + other->m_InsertionPosition.y) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.z - other->m_InsertionPosition.z) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.x - other->m_Cylinder1Position.x) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.y + other->m_Cylinder1Position.y) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.z - other->m_Cylinder1Position.z) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.x - other->m_Cylinder2Position.x) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.y + other->m_Cylinder2Position.y) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.z - other->m_Cylinder2Position.z) > epsilon) return __LINE__;
        break;

    case ZAxis:
        if (fabs(this->m_OriginPosition.x - other->m_OriginPosition.x) > epsilon) return __LINE__;
        if (fabs(this->m_OriginPosition.y - other->m_OriginPosition.y) > epsilon) return __LINE__;
        if (fabs(this->m_OriginPosition.z + other->m_OriginPosition.z) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.x - other->m_InsertionPosition.x) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.y - other->m_InsertionPosition.y) > epsilon) return __LINE__;
        if (fabs(this->m_InsertionPosition.z + other->m_InsertionPosition.z) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.x - other->m_Cylinder1Position.x) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.y - other->m_Cylinder1Position.y) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder1Position.z + other->m_Cylinder1Position.z) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.x - other->m_Cylinder2Position.x) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.y - other->m_Cylinder2Position.y) > epsilon) return __LINE__;
        if (fabs(this->m_Cylinder2Position.z + other->m_Cylinder2Position.z) > epsilon) return __LINE__;
        break;
    }

    // now check for left to right crossover errors
    if (this->m_Name.find(sanityCheckLeft) != std::string::npos)
    {
        if (m_OriginBody->GetName()->find(sanityCheckRight) != std::string::npos) return __LINE__;
        if (m_InsertionBody->GetName()->find(sanityCheckRight) != std::string::npos) return __LINE__;
        if (m_Cylinder1Body->GetName()->find(sanityCheckRight) != std::string::npos) return __LINE__;
        if (m_Cylinder2Body->GetName()->find(sanityCheckRight) != std::string::npos) return __LINE__;
    }
    if (this->m_Name.find(sanityCheckRight) != std::string::npos)
    {
        if (m_OriginBody->GetName()->find(sanityCheckLeft) != std::string::npos) return __LINE__;
        if (m_InsertionBody->GetName()->find(sanityCheckLeft) != std::string::npos) return __LINE__;
        if (m_Cylinder1Body->GetName()->find(sanityCheckLeft) != std::string::npos) return __LINE__;
        if (m_Cylinder2Body->GetName()->find(sanityCheckLeft) != std::string::npos) return __LINE__;
    }

    return 0;
}

#ifdef USE_QT
void TwoCylinderWrapStrap::Draw(SimulationWindow *window)
{
    const int kSides = 128;
    FacetedPolyline *facetedPolyline;
    unsigned int i;

    if (m_simulation->GetTime() != m_LastDrawTime)
    {
        m_LastDrawTime = m_simulation->GetTime();
        std::vector<FacetedObject *>::const_iterator iterFO;
        for (iterFO = m_DrawList.begin(); iterFO != m_DrawList.end(); iterFO++)
            delete *iterFO;
        m_DrawList.clear();

        std::vector<pgd::Vector> polyline;
        for (i = 0; i < (unsigned int)m_NumPathCoordinates; i++)
        {
            polyline.push_back(m_PathCoordinates[i]);
        }
        facetedPolyline = new FacetedPolyline(&polyline, m_Radius, kSides);
        facetedPolyline->SetColour(m_Colour);
        facetedPolyline->setSimulationWindow(window);
        facetedPolyline->SetVisible(m_Visible);
        m_DrawList.push_back(facetedPolyline);

        // calculate the quaternion that rotates from cylinder coordinates to world coordinates
        const double *q = dBodyGetQuaternion(m_Cylinder1Body->GetBodyID());
        pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
        pgd::Quaternion cylinderToWorldQuaternion =  qBody * m_CylinderQuaternion;
        pgd::Vector cylinderVecWorld = pgd::QVRotate(cylinderToWorldQuaternion, pgd::Vector(0, 0, m_CylinderLength / 2));
        // calculate the cylinders world position
        dVector3 position;
        dBodyGetRelPointPos(m_Cylinder1Body->GetBodyID(), m_Cylinder1Position.x, m_Cylinder1Position.y, m_Cylinder1Position.z, position);
        // and draw it
        polyline.clear();
        polyline.push_back(pgd::Vector(position[0] - cylinderVecWorld.x, position[1] - cylinderVecWorld.y, position[2] - cylinderVecWorld.z));
        polyline.push_back(pgd::Vector(position[0] + cylinderVecWorld.x, position[1] + cylinderVecWorld.y, position[2] + cylinderVecWorld.z));
        facetedPolyline = new FacetedPolyline(&polyline, m_Cylinder1Radius, kSides);
        facetedPolyline->SetColour(m_CylinderColour);
        facetedPolyline->setSimulationWindow(window);
        facetedPolyline->SetVisible(m_Visible);
        m_DrawList.push_back(facetedPolyline);

        dBodyGetRelPointPos(m_Cylinder2Body->GetBodyID(), m_Cylinder2Position.x, m_Cylinder2Position.y, m_Cylinder2Position.z, position);
        // and draw it
        polyline.clear();
        polyline.push_back(pgd::Vector(position[0] - cylinderVecWorld.x, position[1] - cylinderVecWorld.y, position[2] - cylinderVecWorld.z));
        polyline.push_back(pgd::Vector(position[0] + cylinderVecWorld.x, position[1] + cylinderVecWorld.y, position[2] + cylinderVecWorld.z));
        facetedPolyline = new FacetedPolyline(&polyline, m_Cylinder2Radius, kSides);
        facetedPolyline->SetColour(m_CylinderColour);
        facetedPolyline->setSimulationWindow(window);
        facetedPolyline->SetVisible(m_Visible);
        m_DrawList.push_back(facetedPolyline);

        if (m_drawMuscleForces)
        {
            for (i = 0; i < m_PointForceList.size(); i++)
            {
                polyline.clear();
                pgd::Vector f = pgd::Vector(m_PointForceList[i]->vector[0], m_PointForceList[i]->vector[1], m_PointForceList[i]->vector[2]) * m_Tension * m_ForceScale;
                polyline.push_back(pgd::Vector(m_PointForceList[i]->point[0], m_PointForceList[i]->point[1], m_PointForceList[i]->point[2]));
                polyline.push_back(pgd::Vector(m_PointForceList[i]->point[0], m_PointForceList[i]->point[1], m_PointForceList[i]->point[2]) + f);
                facetedPolyline = new FacetedPolyline(&polyline, m_ForceRadius, kSides);
                facetedPolyline->SetColour(m_Colour);
                facetedPolyline->setSimulationWindow(window);
                facetedPolyline->SetVisible(m_drawMuscleForces);
                m_DrawList.push_back(facetedPolyline);
            }
        }
    }

    for (i = 0; i < m_DrawList.size(); i++)
        m_DrawList[i]->Draw();
}
#endif
