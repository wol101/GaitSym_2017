/*
 *  Geom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Wrapper class to hold ODE geom

#include <ode/ode.h>

#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif


#include "Geom.h"
#include "PGDMath.h"
#include "DataFile.h"
#include "Simulation.h"
#include "Body.h"
#include "Util.h"
#include "Contact.h"

Geom::Geom()
{
    m_GeomID = 0;
    m_GeomLocation = environment;
    m_CFM = -1; // < 0 is not used
    m_ERP = 2; // > 1 is not used
    m_Bounce = -1; // < 0 is not used
    m_Mu = dInfinity;
    m_Abort = false;
}

Geom::~Geom()
{
    if (m_GeomID) dGeomDestroy(m_GeomID);
}


// these functions set the geom position relative to its body
// these now use the geom offset functions
void Geom::SetBody(dBodyID setBody)
{
    dGeomSetBody(m_GeomID, setBody);
}

dBodyID Geom::GetBody()
{
    return dGeomGetBody(m_GeomID);
}

void Geom::SetPosition (double x, double y, double z)
{
    dGeomSetOffsetPosition(m_GeomID, x, y, z);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void Geom::SetPosition(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;
    dBodyID geomBody = dGeomGetBody(m_GeomID);

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 3)
    {
        std::cerr << "Error in Geom::SetPosition\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(geomBody, pos[0], pos[1], pos[2], result); // convert from world to body
        SetPosition(result[0], result[1], result[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in Geom::SetPosition\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(geomBody, pos[0], pos[1], pos[2], result); // convert from world to body
            SetPosition(result[0], result[1], result[2]);
            return;
        }
        else
        {
            std::cerr << "Error in Geom::SetPosition\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(geomBody, result[0], result[1], result[2], pos); // convert from world to body
    SetPosition(pos[0], pos[1], pos[2]);
}

const double *Geom::GetPosition()
{
    return dGeomGetOffsetPosition(m_GeomID);
}

void Geom::GetWorldPosition(dVector3 p)
{
    const double *relPosition = dGeomGetOffsetPosition(m_GeomID);
    dBodyGetRelPointPos(dGeomGetBody(m_GeomID), relPosition[0], relPosition[1], relPosition[2], p);
    return;
}

void Geom::SetQuaternion(double q0, double q1, double q2, double q3)
{
    dQuaternion q;
    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
    dGeomSetOffsetQuaternion(m_GeomID, q);
}

// parses the quaternion allowing a relative position specified by BODY ID
// note quaternion is (qs,qx,qy,qz)
// s x y z - world coordinates
// bodyName s x y z - position relative to bodyName local coordinate system
void Geom::SetQuaternion(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion, q;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 4)
    {
        std::cerr << "Error in Geom::SetQuaternion\n";
        return; // error condition
    }


    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        dGeomGetQuaternion(m_GeomID, q);
        pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
        Util::GetQuaternion(&lBufPtrs[0], quaternion);
        pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        pgd::Quaternion qLocal = ~qBody * qWorld;
        SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
        return;
    }

    if (count < 5)
    {
        std::cerr << "Error in Geom::SetQuaternion\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            dGeomGetQuaternion(m_GeomID, q);
            pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
            Util::GetQuaternion(&lBufPtrs[1], quaternion);
            pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            pgd::Quaternion qLocal = ~qBody * qWorld;
            SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
            return;
        }
        else
        {
            std::cerr << "Error in Geom::SetQuaternion\n";
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
    dGeomGetQuaternion(m_GeomID, q);
    pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
    pgd::Quaternion qLocal = ~qBody * qWorld;
    SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
}

void Geom::GetQuaternion(dQuaternion q)
{
    dGeomGetOffsetQuaternion(m_GeomID, q);
}

void Geom::GetWorldQuaternion(dQuaternion q)
{
    const double *bodyRotation = dBodyGetQuaternion(dGeomGetBody(m_GeomID));
    dQuaternion relRotation;
    dGeomGetOffsetQuaternion(m_GeomID, relRotation);
    //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
    dQMultiply0 (q, bodyRotation, relRotation);
}

void Geom::SetSpringDamp(double springConstant, double dampingConstant, double integrationStep)
{
    m_ERP = integrationStep * springConstant/(integrationStep * springConstant + dampingConstant);
    m_CFM = 1/(integrationStep * springConstant + dampingConstant);
}

void Geom::SetSpringERP(double springConstant, double ERP, double integrationStep)
{
    m_ERP = ERP;
    m_CFM = ERP / (integrationStep * springConstant);
}

void Geom::Dump()
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
            *m_DumpStream << "Time\tXP\tYP\tZP\tQW\tQX\tQY\tQZ\tNContacts\tBody1\tBody2\tXC\tYC\tZC\tFX1\tFY1\tFZ1\tTX1\tTY1\tTZ1\tFX2\tFY2\tFZ2\tTX2\tTY2\tTZ2\n";
        }
    }


    if (m_DumpStream)
    {

        dQuaternion bodyRotation;
        dGeomGetQuaternion(m_GeomID, bodyRotation);
        const double *relPosition = dGeomGetOffsetPosition(m_GeomID);
        dQuaternion relRotation;
        dGeomGetOffsetQuaternion(m_GeomID, relRotation);

        dVector3 p;
        dQuaternion q;

        // get the position in world coordinates
        dBodyGetRelPointPos(dGeomGetBody(m_GeomID), relPosition[0], relPosition[1], relPosition[2], p);

        //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
        dQMultiply0 (q, bodyRotation, relRotation);

        *m_DumpStream << m_simulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] <<
                "\t" << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] << "\t" <<
                m_ContactList.size();

        dJointFeedback *jointFeedback;
        dBodyID bodyID;
        for (unsigned int i = 0; i < m_ContactList.size(); i++)
        {
            bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 0);
            if (bodyID == GetBody()) // put them the normal way round
            {
                if (bodyID == 0) *m_DumpStream << "\tStatic_Environment\t";
                else *m_DumpStream << "\t" << *((Body *)(dBodyGetData(bodyID)))->GetName() << "\t";
                bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 1);
                if (bodyID == 0) *m_DumpStream << "Static_Environment\t";
                else *m_DumpStream << *((Body *)(dBodyGetData(bodyID)))->GetName() << "\t";

                *m_DumpStream << (*m_ContactList[i]->GetContactPosition())[0] << "\t" <<
                        (*m_ContactList[i]->GetContactPosition())[1] << "\t" <<
                        (*m_ContactList[i]->GetContactPosition())[2] << "\t";

                jointFeedback = m_ContactList[i]->GetJointFeedback();
                *m_DumpStream <<
                        jointFeedback->f1[0] << "\t" << jointFeedback->f1[1] << "\t" << jointFeedback->f1[2] << "\t" <<
                        jointFeedback->t1[0] << "\t" << jointFeedback->t1[1] << "\t" << jointFeedback->t1[2] << "\t" <<
                        jointFeedback->f2[0] << "\t" << jointFeedback->f2[1] << "\t" << jointFeedback->f2[2] << "\t" <<
                        jointFeedback->t2[0] << "\t" << jointFeedback->t2[1] << "\t" << jointFeedback->t2[2];
            }
            else // reverse the order since our body is second
            {
                bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 1);
                if (bodyID == 0) *m_DumpStream << "Static_Environment\t";
                else *m_DumpStream << *((Body *)(dBodyGetData(bodyID)))->GetName() << "\t";
                bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 0);
                if (bodyID == 0) *m_DumpStream << "\tStatic_Environment\t";
                else *m_DumpStream << "\t" << *((Body *)(dBodyGetData(bodyID)))->GetName() << "\t";

                *m_DumpStream << (*m_ContactList[i]->GetContactPosition())[0] << "\t" <<
                        (*m_ContactList[i]->GetContactPosition())[1] << "\t" <<
                        (*m_ContactList[i]->GetContactPosition())[2] << "\t";

                jointFeedback = m_ContactList[i]->GetJointFeedback();
                *m_DumpStream <<
                        jointFeedback->f2[0] << "\t" << jointFeedback->f2[1] << "\t" << jointFeedback->f2[2] << "\t" <<
                        jointFeedback->t2[0] << "\t" << jointFeedback->t2[1] << "\t" << jointFeedback->t2[2] << "\t" <<
                        jointFeedback->f1[0] << "\t" << jointFeedback->f1[1] << "\t" << jointFeedback->f1[2] << "\t" <<
                        jointFeedback->t1[0] << "\t" << jointFeedback->t1[1] << "\t" << jointFeedback->t1[2];
        }
        }
        *m_DumpStream << "\n";
    }
}

