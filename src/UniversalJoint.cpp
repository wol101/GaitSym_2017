/*
 *  UniversalJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/12/2010.
 *  Copyright 2010 Bill Sellers. All rights reserved.
 *
 */

// Note this joint is implemented as an ODE Hinge2 joint so that the requirement for perpendicular joint axes is relaxed

#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include <ode/ode.h>

#include "UniversalJoint.h"
#include "DataFile.h"
#include "Body.h"
#include "Simulation.h"
#include "DebugControl.h"

#ifdef USE_QT
#include "FacetedConicSegment.h"
#endif

UniversalJoint::UniversalJoint(dWorldID worldID) : Joint()
{
    m_JointID = dJointCreateHinge2(worldID, 0);
    dJointSetData(m_JointID, this);

    dJointSetFeedback(m_JointID, &m_JointFeedback);

    m_StartAngleReference1 = 0;
    m_StartAngleReference2 = 0;
}

void UniversalJoint::SetUniversalAnchor (double x, double y, double z)
{
    dJointSetHinge2Anchor(m_JointID, x, y, z);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void UniversalJoint::SetUniversalAnchor(const char *buf)
{
    int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos;

    strcpy(lBuf, buf);
    int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
    if (count < 3)
    {
        std::cerr << "Error in UniversalJoint::SetUniversalAnchor\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetUniversalAnchor(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in UniversalJoint::SetUniversalAnchor\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetUniversalAnchor(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in UniversalJoint::SetUniversalAnchor\n";
            return; // error condition
        }
    }
    dVector3 result;
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos (theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    SetUniversalAnchor(result[0], result[1], result[2]);
}

void UniversalJoint::SetUniversalAxis1(double x, double y, double z)
{
    dVector3 v;
    v[0] = x; v[1] = y; v[2] = z;
    dNormalize3(v);
    dJointSetHinge2Axis1(m_JointID, v[0], v[1], v[2]);
}

void UniversalJoint::SetUniversalAxis2(double x, double y, double z)
{
    dVector3 v;
    v[0] = x; v[1] = y; v[2] = z;
    dNormalize3(v);
    dJointSetHinge2Axis1(m_JointID, v[0], v[1], v[2]);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void UniversalJoint::SetUniversalAxis1(const char *buf)
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
        std::cerr << "Error in UniversalJoint::SetUniversalAxis\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetUniversalAxis1(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in UniversalJoint::SetUniversalAxis\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetUniversalAxis1(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in UniversalJoint::SetUniversalAxis\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    SetUniversalAxis1(result[0], result[1], result[2]);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void UniversalJoint::SetUniversalAxis2(const char *buf)
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
        std::cerr << "Error in UniversalJoint::SetUniversalAxis\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetUniversalAxis2(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in UniversalJoint::SetUniversalAxis\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetUniversalAxis2(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in UniversalJoint::SetUniversalAxis\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    SetUniversalAxis2(result[0], result[1], result[2]);
}

void UniversalJoint::SetStartAngleReference1(double startAngleReference)
{
    m_StartAngleReference1 = startAngleReference;
}

void UniversalJoint::SetStartAngleReference2(double startAngleReference)
{
    m_StartAngleReference2 = startAngleReference;
}

void UniversalJoint::GetUniversalAnchor(dVector3 result)
{
    dJointGetHinge2Anchor(m_JointID, result);
}

void UniversalJoint::GetUniversalAnchor2(dVector3 result)
{
    dJointGetHinge2Anchor2(m_JointID, result);
}

void UniversalJoint::GetUniversalAxis1(dVector3 result)
{
    dJointGetHinge2Axis1(m_JointID, result);
}

void UniversalJoint::GetUniversalAxis2(dVector3 result)
{
    dJointGetHinge2Axis2(m_JointID, result);
}

double UniversalJoint::GetUniversalAngle1()
{
    return dJointGetHinge2Angle1(m_JointID) + m_StartAngleReference1;
}

double UniversalJoint::GetUniversalAngle2()
{
    // this isn't implemented in ODE yet
    // return dJointGetHinge2Angle2(m_JointID) + m_StartAngleReference2;
    return 0;
}

double UniversalJoint::GetUniversalAngle1Rate()
{
    return dJointGetHinge2Angle1Rate(m_JointID);
}

double UniversalJoint::GetUniversalAngle2Rate()
{
    return dJointGetHinge2Angle2Rate(m_JointID);
}

void UniversalJoint::Dump()
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
            *m_DumpStream << "Time\tXP\tYP\tZP\tXA1\tYA1\tZA1\tAngle1\tAngleRate1\tXA2\tYA2\tZA2\tAngle2\tAngleRate2\tFX1\tFY1\tFZ1\tTX1\tTY1\tTZ1\tFX2\tFY2\tFZ2\tTX2\tTY2\tTZ2\n";
        }
    }


    if (m_DumpStream)
    {
        dVector3 p, a1, a2;
        GetUniversalAnchor(p);
        GetUniversalAxis1(a1);
        GetUniversalAxis2(a2);

        *m_DumpStream << m_simulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" <<
                a1[0] << "\t" << a1[1] << "\t" << a1[2] << "\t" << GetUniversalAngle1() << "\t" << GetUniversalAngle1Rate() << "\t" <<
                a2[0] << "\t" << a2[1] << "\t" << a2[2] << "\t" << GetUniversalAngle2() << "\t" << GetUniversalAngle2Rate() << "\t" <<
                m_JointFeedback.f1[0] << "\t" << m_JointFeedback.f1[1] << "\t" << m_JointFeedback.f1[2] << "\t" <<
                m_JointFeedback.t1[0] << "\t" << m_JointFeedback.t1[1] << "\t" << m_JointFeedback.t1[2] << "\t" <<
                m_JointFeedback.f2[0] << "\t" << m_JointFeedback.f2[1] << "\t" << m_JointFeedback.f2[2] << "\t" <<
                m_JointFeedback.t2[0] << "\t" << m_JointFeedback.t2[1] << "\t" << m_JointFeedback.t2[2] <<
                "\n";
    }
}

#ifdef USE_QT
void UniversalJoint::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        m_physRep = new FacetedConicSegment(m_AxisSize[0] * 2, m_AxisSize[0] / 10, m_AxisSize[0] / 10, 128, 0, 0, -m_AxisSize[0]);
        m_physRep->setSimulationWindow(window);
        m_physRep->SetColour(m_Colour);
        m_physRep2 = new FacetedConicSegment(m_AxisSize[0] * 2, m_AxisSize[0] / 10, m_AxisSize[0] / 10, 128, 0, 0, -m_AxisSize[0]);
        m_physRep2->setSimulationWindow(window);
        m_physRep2->SetColour(m_Colour);
    }

    dVector3 anchor;
    dVector3 axis;
    GetUniversalAnchor(anchor);
    GetUniversalAxis1(axis);

    m_physRep->SetColour(m_Colour);
    m_physRep->SetDisplayRotationFromAxis(axis[0], axis[1], axis[2], false);
    m_physRep->SetDisplayPosition(anchor[0], anchor[1], anchor[2]);
    m_physRep->Draw();

    GetUniversalAxis2(axis);

    m_physRep2->SetColour(m_Colour);
    m_physRep2->SetDisplayRotationFromAxis(axis[0], axis[1], axis[2], false);
    m_physRep2->SetDisplayPosition(anchor[0], anchor[1], anchor[2]);
    m_physRep2->Draw();

}
#endif
