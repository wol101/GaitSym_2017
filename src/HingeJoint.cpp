/*
 *  HingeJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <iostream>
#include <stdlib.h>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include "HingeJoint.h"
#include "DataFile.h"
#include "Body.h"
#include "Simulation.h"
#include "DebugControl.h"
#include "PGDMath.h"

#ifdef USE_QT
#include "FacetedConicSegment.h"
#endif

HingeJoint::HingeJoint(dWorldID worldID) : Joint()
{
    m_JointID = dJointCreateHinge(worldID, 0);
    dJointSetData(m_JointID, this);

    dJointSetFeedback(m_JointID, &m_JointFeedback);
    m_StartAngleReference = 0;
    m_HiStopTorqueLimit = dInfinity;
    m_LoStopTorqueLimit = -dInfinity;
    m_axisTorque = 0;

    m_axisTorqueList = 0;
    m_axisTorqueTotal = 0;
    m_axisTorqueMean = 0;
    m_axisTorqueIndex = 0;
    m_axisTorqueWindow = 0;

}

HingeJoint::~HingeJoint()
{
    if (m_axisTorqueList) delete [] m_axisTorqueList;
}

void HingeJoint::SetHingeAnchor (double x, double y, double z)
{
    dJointSetHingeAnchor(m_JointID, x, y, z);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void HingeJoint::SetHingeAnchor(const char *buf)
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
        std::cerr << "Error in HingeJoint::SetHingeAnchor\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dJointSetHingeAnchor(m_JointID, pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in HingeJoint::SetHingeAnchor\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dJointSetHingeAnchor(m_JointID, pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in HingeJoint::SetHingeAnchor\n";
            return; // error condition
        }
    }
    dVector3 result;
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos (theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    dJointSetHingeAnchor(m_JointID, result[0], result[1], result[2]);
}

void HingeJoint::SetHingeAxis(double x, double y, double z)
{
    dVector3 v;
    v[0] = x; v[1] = y; v[2] = z;
    dNormalize3(v);
    dJointSetHingeAxis(m_JointID, v[0], v[1], v[2]);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void HingeJoint::SetHingeAxis(const char *buf)
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
        std::cerr << "Error in HingeJoint::SetHingeAxis\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetHingeAxis(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in HingeJoint::SetHingeAxis\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetHingeAxis(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in HingeJoint::SetHingeAxis\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    SetHingeAxis(result[0], result[1], result[2]);
}

void HingeJoint::GetHingeAnchor(dVector3 result)
{
    dJointGetHingeAnchor(m_JointID, result);
}

void HingeJoint::GetHingeAnchor2(dVector3 result)
{
    dJointGetHingeAnchor2(m_JointID, result);
}

void HingeJoint::GetHingeAxis(dVector3 result)
{
    dJointGetHingeAxis(m_JointID, result);
}

double HingeJoint::GetHingeAngle()
{
    return dJointGetHingeAngle(m_JointID) + m_StartAngleReference;
}

double HingeJoint::GetHingeAngleRate()
{
    return dJointGetHingeAngleRate(m_JointID);
}

void HingeJoint::SetStartAngleReference(double startAngleReference)
{
    m_StartAngleReference = startAngleReference;
}

void HingeJoint::SetJointStops(double loStop, double hiStop)
{
    if (loStop >= hiStop)
    {
        std::cerr << "Error in HingeJoint::SetJointStops loStop >= hiStop\n";
        throw(__LINE__);
    }

    // correct for m_StartAngleReference
    loStop -= m_StartAngleReference;
    hiStop -= m_StartAngleReference;

    if (loStop < -M_PI) loStop = -dInfinity;
    if (hiStop > M_PI) hiStop = dInfinity;

    // note there is safety feature that stops setting incompatible low and high
    // stops which can cause difficulties. The safe option is to set them twice.

    dJointSetHingeParam(m_JointID, dParamLoStop, loStop);
    dJointSetHingeParam(m_JointID, dParamHiStop, hiStop);
    dJointSetHingeParam(m_JointID, dParamLoStop, loStop);
    dJointSetHingeParam(m_JointID, dParamHiStop, hiStop);

    // we don't want bouncy stops
    dJointSetHingeParam(m_JointID, dParamBounce, 0);
}

void HingeJoint::CalculateStopTorque()
{
#ifdef USE_JOINT_ANGLES_FOR_STOP_TORQUE
    // this is the approximate stop torque from the amount past the limit that the joint is
    // it doesn't take into account damping and probably isn't actualy the value used anyway

    double angle = dJointGetHingeAngle(m_JointID);
    double loStop = dJointGetHingeParam (m_JointID, dParamLoStop);
    double hiStop = dJointGetHingeParam (m_JointID, dParamHiStop);
    double t = 0; // calculated torque from error
    double ERP, CFM;
    double kp = 0;
    // double kd = 0; // decided not to use damping in these calculations
    if (angle > hiStop)
    {
        ERP = dJointGetHingeParam (m_JointID, dParamStopERP);
        CFM = dJointGetHingeParam (m_JointID, dParamStopCFM);
        kp = ERP / (CFM * m_simulation->GetTimeIncrement());
        //kd = (1 - ERP) / CFM;
        //t = (hiStop - angle) * kp - GetHingeAngleRate() * kd;
        t = (hiStop - angle) * kp;
    }
    else if (angle < loStop)
    {
        ERP = dJointGetHingeParam (m_JointID, dParamStopERP);
        CFM = dJointGetHingeParam (m_JointID, dParamStopCFM);
        kp = ERP / (CFM * m_simulation->GetTimeIncrement());
        //kd = (1 - ERP) / CFM;
        //t = (loStop - angle) * kp - GetHingeAngleRate() * kd;
        t = (loStop - angle) * kp;
    }

    if (gDebug == HingeJointDebug)
    {
        if (DebugFilters("GetStopTorque", m_Name))
            // std::cerr << m_Name << " angle " << GetHingeAngle() << " dangle " << GetHingeAngleRate() << " kp " << kp << " kd " << kd << " t " << t << "\n";
            std::cerr << m_Name << " loStop " <<  loStop + m_StartAngleReference <<  " hiStop " <<  hiStop + m_StartAngleReference <<  " angle " << GetHingeAngle() << " kp " << kp << " t " << t << "\n";
    }
#endif

    if (m_simulation->GetTime() <= 0)
    {
        m_axisTorque = 0;
        return;
    }


    // now do it properly
    // first of all we need to convert the forces and torques into the joint local coordinate system

    // the force feedback is at the CM for fixed joints
    // first we need to move it to the joint position

    // calculate the offset of the joint anchor from the CM
    dVector3 jointAnchor;
    dJointGetHingeAnchor(m_JointID, jointAnchor);
    dBodyID bodyID = dJointGetBody(m_JointID, 0);
    pgd::Vector worldForceOffset;
    if (bodyID)
    {
        const double *bodyPosition = dBodyGetPosition(bodyID);
        worldForceOffset = pgd::Vector(jointAnchor[0] - bodyPosition[0], jointAnchor[1] - bodyPosition[1], jointAnchor[2] - bodyPosition[2]);
    }
    else
    {
        worldForceOffset = pgd::Vector(jointAnchor[0], jointAnchor[1], jointAnchor[2]);
    }

    // now the linear components of m_JointFeedback will generate a torque if applied at this position
    // torque = r x f
    pgd::Vector forceCM(m_JointFeedback.f1[0], m_JointFeedback.f1[1], m_JointFeedback.f1[2]);
    pgd::Vector addedTorque = worldForceOffset ^ forceCM;

    pgd::Vector torqueCM(m_JointFeedback.t1[0], m_JointFeedback.t1[1], m_JointFeedback.t1[2]);
    pgd::Vector torqueJointAnchor = torqueCM - addedTorque;

    double torqueScalar = torqueJointAnchor.Magnitude();
    if (torqueScalar == 0)
    {
        m_axisTorque = 0;
        return;
    }

    pgd::Vector torqueAxis = torqueJointAnchor / torqueScalar;

    // so the torque around the hinge axis should be: torqueScalar * (hingeAxis .dot. torqueAxis)
    dVector3 result;
    dJointGetHingeAxis(m_JointID, result);
    pgd::Vector hingeAxis(result[0], result[1], result[2]);
    m_axisTorque = torqueScalar * (hingeAxis * torqueAxis);

    if (m_axisTorqueWindow < 2)
    {
        m_axisTorqueMean = m_axisTorque;
    }
    else
    {
        m_axisTorqueIndex++;
        if (m_axisTorqueIndex >= m_axisTorqueWindow) m_axisTorqueIndex = 0;

        m_axisTorqueTotal -= m_axisTorqueList[m_axisTorqueIndex];
        m_axisTorqueTotal += m_axisTorque;
        m_axisTorqueList[m_axisTorqueIndex] = m_axisTorque;
        m_axisTorqueMean = m_axisTorqueTotal / m_axisTorqueWindow;
    }
}

int HingeJoint::TestLimits()
{
    if (m_axisTorqueMean < m_LoStopTorqueLimit) return -1;
    if (m_axisTorqueMean > m_HiStopTorqueLimit) return 1;
    return 0;
}

void HingeJoint::SetStopTorqueWindow(int window)
{
    if (m_axisTorqueList) delete [] m_axisTorqueList;

    if (window > 1)
    {
        m_axisTorqueList = new double[window];
        memset(m_axisTorqueList, 0, sizeof(double) * window);
        m_axisTorqueWindow = window;
    }
    else
    {
        m_axisTorqueList = 0;
        m_axisTorqueWindow = 0;
    }
    m_axisTorqueTotal = 0;
    m_axisTorqueMean = 0;
    m_axisTorqueIndex = 0;
}

void HingeJoint::SetStopCFM(double cfm)
{
    dJointSetHingeParam (m_JointID, dParamStopCFM, cfm);
}

void HingeJoint::SetStopERP(double erp)
{
    dJointSetHingeParam (m_JointID, dParamStopERP, erp);
}

void HingeJoint::SetStopSpringDamp(double springConstant, double dampingConstant, double integrationStep)
{
    double ERP = integrationStep * springConstant/(integrationStep * springConstant + dampingConstant);
    double CFM = 1/(integrationStep * springConstant + dampingConstant);
    SetStopERP(ERP);
    SetStopCFM(CFM);
}

void HingeJoint::SetStopSpringERP(double springConstant, double ERP, double integrationStep)
{
    double CFM = ERP / (integrationStep * springConstant);
    SetStopERP(ERP);
    SetStopCFM(CFM);
}

void HingeJoint::SetStopBounce(double bounce)
{
    dJointSetHingeParam (m_JointID, dParamBounce, bounce);
}

void HingeJoint::Update()
{
    CalculateStopTorque();
}

void HingeJoint::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "HingeJoint::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tXP\tYP\tZP\tXA\tYA\tZA\tAngle\tAngleRate\tFX1\tFY1\tFZ1\tTX1\tTY1\tTZ1\tFX2\tFY2\tFZ2\tTX2\tTY2\tTZ2\tStopTorque\n";
        }
    }


    if (m_DumpStream)
    {
        dVector3 p, a;
        GetHingeAnchor(p);
        GetHingeAxis(a);

        *m_DumpStream << m_simulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" <<
                         a[0] << "\t" << a[1] << "\t" << a[2] << "\t" << GetHingeAngle() << "\t" << GetHingeAngleRate() << "\t" <<
                         m_JointFeedback.f1[0] << "\t" << m_JointFeedback.f1[1] << "\t" << m_JointFeedback.f1[2] << "\t" <<
                         m_JointFeedback.t1[0] << "\t" << m_JointFeedback.t1[1] << "\t" << m_JointFeedback.t1[2] << "\t" <<
                         m_JointFeedback.f2[0] << "\t" << m_JointFeedback.f2[1] << "\t" << m_JointFeedback.f2[2] << "\t" <<
                         m_JointFeedback.t2[0] << "\t" << m_JointFeedback.t2[1] << "\t" << m_JointFeedback.t2[2] << "\t" <<
                         m_axisTorque <<
                         "\n";
    }
}

#ifdef USE_QT
void HingeJoint::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        m_physRep = new FacetedConicSegment(m_AxisSize[0] * 2, m_AxisSize[0] / 10, m_AxisSize[0] / 10, 128, 0, 0, -m_AxisSize[0]);
        m_physRep->setSimulationWindow(window);
        m_physRep->SetColour(m_Colour);
    }

    dVector3 anchor;
    dVector3 axis;
    dJointGetHingeAnchor(m_JointID, anchor);
    dJointGetHingeAxis(m_JointID, axis);

    m_physRep->SetColour(m_Colour);
    m_physRep->SetDisplayRotationFromAxis(axis[0], axis[1], axis[2], false);
    m_physRep->SetDisplayPosition(anchor[0], anchor[1], anchor[2]);
    m_physRep->Draw();

}
#endif

