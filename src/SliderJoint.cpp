/*
 *  SliderJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 25/05/2012.
 *  Copyright 2012 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <iostream>
#include <cmath>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include "SliderJoint.h"
#include "Simulation.h"
#include "Body.h"

#ifdef USE_QT
#include "FacetedConicSegment.h"
#endif

SliderJoint::SliderJoint(dWorldID worldID) : Joint()
{
    m_JointID = dJointCreateSlider(worldID, 0);
    dJointSetData(m_JointID, this);

    dJointSetFeedback(m_JointID, &m_JointFeedback);
    m_StartDistanceReference = 0;
}

SliderJoint::~SliderJoint()
{
}

void SliderJoint::SetSliderAxis(double x, double y, double z)
{
    dVector3 v;
    v[0] = x; v[1] = y; v[2] = z;
    dNormalize3(v);
    dJointSetSliderAxis(m_JointID, v[0], v[1], v[2]);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void SliderJoint::SetSliderAxis(const char *buf)
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
        std::cerr << "Error in SliderJoint::SetSliderAxis\n";
        return; // error condition
    }

    if (isalpha((int)*lBufPtrs[0]) == 0)
    {
        for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        SetSliderAxis(pos[0], pos[1], pos[2]);
        return;
    }

    if (count < 4)
    {
        std::cerr << "Error in SliderJoint::SetSliderAxis\n";
        return; // error condition
    }
    Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
    if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            SetSliderAxis(pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in SliderJoint::SetSliderAxis\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyVectorToWorld(theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    SetSliderAxis(result[0], result[1], result[2]);
}

void SliderJoint::GetSliderAxis(dVector3 result)
{
    dJointGetSliderAxis(m_JointID, result);
}

double SliderJoint::GetSliderDistance()
{
    return dJointGetSliderPosition(m_JointID) + m_StartDistanceReference;
}

double SliderJoint::GetSliderDistanceRate()
{
    return dJointGetSliderPositionRate(m_JointID);
}

void SliderJoint::SetStartDistanceReference(double startDistanceReference)
{
    m_StartDistanceReference = startDistanceReference;
}

void SliderJoint::SetJointStops(double loStop, double hiStop)
{
    if (loStop >= hiStop) throw(__LINE__);

    // correct for m_StartDistanceReference
    loStop -= m_StartDistanceReference;
    hiStop -= m_StartDistanceReference;

    // note there is safety feature that stops setting incompatible low and high
    // stops which can cause difficulties. The safe option is to set them twice.

    dJointSetSliderParam(m_JointID, dParamLoStop, loStop);
    dJointSetSliderParam(m_JointID, dParamHiStop, hiStop);
    dJointSetSliderParam(m_JointID, dParamLoStop, loStop);
    dJointSetSliderParam(m_JointID, dParamHiStop, hiStop);

    // we don't want bouncy stops
    dJointSetSliderParam(m_JointID, dParamBounce, 0);
}

void SliderJoint::SetStopCFM(double cfm)
{
    dJointSetSliderParam (m_JointID, dParamStopCFM, cfm);
}

void SliderJoint::SetStopERP(double erp)
{
    dJointSetSliderParam (m_JointID, dParamStopERP, erp);
}

void SliderJoint::SetStopSpringDamp(double springConstant, double dampingConstant, double integrationStep)
{
    double ERP = integrationStep * springConstant/(integrationStep * springConstant + dampingConstant);
    double CFM = 1/(integrationStep * springConstant + dampingConstant);
    SetStopERP(ERP);
    SetStopCFM(CFM);
}

void SliderJoint::SetStopSpringERP(double springConstant, double ERP, double integrationStep)
{
    double CFM = ERP / (integrationStep * springConstant);
    SetStopERP(ERP);
    SetStopCFM(CFM);
}

void SliderJoint::SetStopBounce(double bounce)
{
    dJointSetSliderParam (m_JointID, dParamBounce, bounce);
}

void SliderJoint::Update()
{
}

void SliderJoint::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "SliderJoint::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tXA\tYA\tZA\tDistance\tDistanceRate\tFX1\tFY1\tFZ1\tTX1\tTY1\tTZ1\tFX2\tFY2\tFZ2\tTX2\tTY2\tTZ2\n";
        }
    }


    if (m_DumpStream)
    {
        dVector3 a;
        GetSliderAxis(a);

        *m_DumpStream << m_simulation->GetTime() << "\t" <<
                         a[0] << "\t" << a[1] << "\t" << a[2] << "\t" << GetSliderDistance() << "\t" << GetSliderDistanceRate() << "\t" <<
                         m_JointFeedback.f1[0] << "\t" << m_JointFeedback.f1[1] << "\t" << m_JointFeedback.f1[2] << "\t" <<
                         m_JointFeedback.t1[0] << "\t" << m_JointFeedback.t1[1] << "\t" << m_JointFeedback.t1[2] << "\t" <<
                         m_JointFeedback.f2[0] << "\t" << m_JointFeedback.f2[1] << "\t" << m_JointFeedback.f2[2] << "\t" <<
                         m_JointFeedback.t2[0] << "\t" << m_JointFeedback.t2[1] << "\t" << m_JointFeedback.t2[2] << "\t" <<
                         "\n";
    }
}

#ifdef USE_QT
void SliderJoint::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
    }

    if (GetBody1() != 0 && GetBody2() != 0) // this is a hack because the current implementation does not work with a world fixed joint
    {

        dBodyID b0 = GetBody1()->GetBodyID();
        dBodyID b1 = GetBody2()->GetBodyID();

        if (b0 == 0 && b1 == 0) return;

        double length = GetSliderDistance();
        if (fabs(length) < m_AxisSize[0] / 100) length = copysign(m_AxisSize[0] / 100, length);

        dVector3 axis;
        dJointGetSliderAxis(m_JointID, axis);


        pgd::Vector position;
        pgd::Vector pos0, pos1;
        const double *p;
        if (b0 == 0)
        {
            p = dBodyGetPosition(b1);
            position = pgd::Vector(p[0], p[1], p[2]) + pgd::Vector(axis[0], axis[1], axis[2]) * (0.5 * length);
        }
        else if (b1 == 0)
        {
            p = dBodyGetPosition(b0);
            position = pgd::Vector(p[0], p[1], p[2]) + pgd::Vector(axis[0], axis[1], axis[2]) * (0.5 * length);
        }
        else
        {
            p = dBodyGetPosition(b0);
            pos0 = pgd::Vector(p[0], p[1], p[2]);
            p = dBodyGetPosition(b1);
            pos1 = pgd::Vector(p[0], p[1], p[2]);
            position = (pos1 - pos0) / 2;
        }

        if (m_physRep) delete m_physRep;
        m_physRep = new FacetedConicSegment(length, m_AxisSize[0] / 10, m_AxisSize[0] / 10, 128, 0, 0, -length / 2);
        m_physRep->setSimulationWindow(window);
        m_physRep->SetColour(m_Colour);
        m_physRep->SetDisplayRotationFromAxis(axis[0], axis[1], axis[2], false);
        m_physRep->SetDisplayPosition(position.x, position.y, position.z);
        m_physRep->Draw();
    }
}
#endif
