/*
 *  Marker.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 22/08/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif


#include "Marker.h"
#include "PGDMath.h"
#include "DataFile.h"
#include "Simulation.h"
#include "Body.h"
#include "Util.h"
#ifdef USE_QT
#include "FacetedSphere.h"
#include "SimulationWindow.h"
#endif

Marker::Marker()
{
    mBody = 0;
    mQuaternion.n = 1; // rest of the quaternion is set to zero already
#ifdef USE_QT
    mRadius = 0.005;
#endif

}

Marker::~Marker()
{
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void Marker::SetPosition(const char *buf)
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
        std::cerr << "Error in Marker::SetPosition\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        if (mBody)
        {
            dBodyGetPosRelPoint(mBody, pos[0], pos[1], pos[2], result); // convert from world to body
            SetPosition(result[0], result[1], result[2]);
        }
        else
        {
            SetPosition(pos[0], pos[1], pos[2]);
        }
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Marker::SetPosition\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            if (mBody)
            {
                dBodyGetPosRelPoint(mBody, pos[0], pos[1], pos[2], result); // convert from world to body
                SetPosition(result[0], result[1], result[2]);
            }
            else
            {
                SetPosition(pos[0], pos[1], pos[2]);
            }
            return;
        }
        else
        {
            std::cerr << "Error in Marker::SetPosition\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    if (mBody)
    {
        dBodyGetPosRelPoint(mBody, result[0], result[1], result[2], pos); // convert from world to body
        SetPosition(pos[0], pos[1], pos[2]);
    }
    else
    {
        SetPosition(result[0], result[1], result[2]);
    }
}

// parses the quaternion allowing a relative position specified by BODY ID
// note quaternion is (qs,qx,qy,qz)
// s x y z - world coordinates
// bodyName s x y z - position relative to bodyName local coordinate system
void Marker::SetQuaternion(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion;
    const double *q;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 4)
    {
        std::cerr << "Error in Marker::SetQuaternion\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        if (mBody)
        {
            q = dBodyGetQuaternion(mBody);
            pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
            Util::GetQuaternion(&lBufPtrs[0], quaternion);
            pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            pgd::Quaternion qLocal = ~qBody * qWorld;
            SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
        }
        else
        {
            Util::GetQuaternion(&lBufPtrs[0], quaternion);
            SetQuaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        }
        return;
    }

    if (count < 5)
    {
        std::cerr << "Error in Marker::SetQuaternion\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            if (mBody)
            {
                q = dBodyGetQuaternion(mBody);
                pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
                Util::GetQuaternion(&lBufPtrs[1], quaternion);
                pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
                pgd::Quaternion qLocal = ~qBody * qWorld;
                SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
            }
            else
            {
                Util::GetQuaternion(&lBufPtrs[0], quaternion);
                SetQuaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            }
            return;
        }
        else
        {
            std::cerr << "Error in Marker::SetQuaternion\n";
            return; // error condition
        }
    }
    // first get world quaternion
    const double *q2 = theBody->GetQuaternion();
    pgd::Quaternion qBody1(q2[0], q2[1], q2[2], q2[3]);
    for (i = 0; i < 4; i++) quaternion[i] = strtod(lBufPtrs[i + 1], 0);
    pgd::Quaternion qBody2(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    pgd::Quaternion qWorld = qBody1 * qBody2;

    // then set the local quaternion
    if (mBody)
    {
        q = dBodyGetQuaternion(mBody);
        pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
        pgd::Quaternion qLocal = ~qBody * qWorld;
        SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
    }
    else
    {
        SetQuaternion(qWorld.n, qWorld.v.x, qWorld.v.y, qWorld.v.z);
    }
}

pgd::Vector Marker::GetWorldPosition()
{
    if (mBody)
    {
        // get the position in world coordinates
        dVector3 p;
        dBodyGetRelPointPos(mBody, mPosition.x, mPosition.y, mPosition.z, p);
        return pgd::Vector(p[0], p[1], p[2]);
    }
    else
    {
        return mPosition;
    }
}

pgd::Quaternion Marker::GetWorldQuaternion()
{
    if (mBody)
    {
        const double *bodyRotation = dBodyGetQuaternion(mBody);
        pgd::Quaternion bodyQuaternion(bodyRotation[0], bodyRotation[1], bodyRotation[2], bodyRotation[3]);
        return bodyQuaternion * mQuaternion;
    }
    else
    {
        return mQuaternion;
    }
}

void Marker::Dump()
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
            *m_DumpStream << "Time\tXP\tYP\tZP\tQW\tQX\tQY\tQZ\n";
        }
    }


    if (m_DumpStream)
    {
        pgd::Vector p = GetWorldPosition();
        pgd::Quaternion q = GetWorldQuaternion();

        *m_DumpStream << m_simulation->GetTime() << "\t" << p.x << "\t" << p.y << "\t" << p.z <<
                "\t" << q.n << "\t" << q.v.x << "\t" << q.v.y << "\t" << q.v.z << "\n";
    }
}

#ifdef USE_QT
void Marker::Draw(SimulationWindow *window)
{
    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        m_physRep = new FacetedSphere(mRadius, 4);
        m_physRep->SetColour(m_Colour);
        m_physRep->setSimulationWindow(window);
    }

    // get the world position
    pgd::Vector p = GetWorldPosition();
    pgd::Quaternion q = GetWorldQuaternion();
    // but I need ODE style quaternions
    dQuaternion qq;
    qq[0] = q.n;
    qq[1] = q.v.x;
    qq[2] = q.v.y;
    qq[3] = q.v.z;

    m_physRep->SetDisplayPosition(p.x, p.y, p.z);
    m_physRep->SetDisplayRotationFromQuaternion(qq);
    m_physRep->Draw();
}
#endif
