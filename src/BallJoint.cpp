/*
 *  BallJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/12/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include <assert.h>
#include <stdlib.h>

#include <ode/ode.h>

#include "BallJoint.h"
#include "DataFile.h"
#include "Body.h"
#include "Simulation.h"
#include "DebugControl.h"
#include "Util.h"

#ifdef USE_QT
#include "FacetedSphere.h"
#endif

BallJoint::BallJoint(dWorldID worldID, int mode) : Joint()
{
    // ball joint
    m_JointID = dJointCreateBall(worldID, 0);
    dJointSetData(m_JointID, this);
    dJointSetFeedback(m_JointID, &m_JointFeedback);

    // angular motor
    m_MotorJointID = dJointCreateAMotor (worldID, 0);
    dJointSetFeedback(m_MotorJointID, &m_MotorJointFeedback);

    m_Mode = mode;
}

void BallJoint::Attach(Body *body1, Body *body2)
{
    assert(body1 != 0 || body2 != 0);
    m_Body1 = body1;
    m_Body2 = body2;
    if (m_Body1 == 0)
    {
        dJointAttach(m_JointID, 0, m_Body2->GetBodyID());
        dJointAttach(m_MotorJointID, 0, m_Body2->GetBodyID());
    }
    else if (m_Body2 == 0)
    {
        dJointAttach(m_JointID, m_Body1->GetBodyID(), 0);
        dJointAttach(m_MotorJointID, m_Body1->GetBodyID(), 0);
    }
    else
    {
        dJointAttach(m_JointID, m_Body1->GetBodyID(), m_Body2->GetBodyID());
        dJointAttach(m_MotorJointID, m_Body1->GetBodyID(), m_Body2->GetBodyID());
    }
}

void BallJoint::SetBallAnchor (double x, double y, double z)
{
    dJointSetBallAnchor(m_JointID, x, y, z);
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
BallJoint::SetBallAnchor(const char *buf)
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
        std::cerr << "Error in BallJoint::SetBallAnchor\n";
        return; // error condition
    }

        if (isalpha((int)*lBufPtrs[0]) == 0)
        {
                for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dJointSetBallAnchor(m_JointID, pos[0], pos[1], pos[2]);
                return;
        }

        if (count < 4)
    {
        std::cerr << "Error in BallJoint::SetBallAnchor\n";
        return; // error condition
    }
        Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
        if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dJointSetBallAnchor(m_JointID, pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in BallJoint::SetBallAnchor\n";
            return; // error condition
        }
    }
    dVector3 result;
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos (theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    dJointSetBallAnchor(m_JointID, result[0], result[1], result[2]);
}

void BallJoint::GetBallAnchor(dVector3 result)
{
    dJointGetBallAnchor(m_JointID, result);
}

void BallJoint::GetBallAnchor2(dVector3 result)
{
    dJointGetBallAnchor2(m_JointID, result);
}

// this routine sets up the axes for the joints
// only axis0 and axis2 are used for m_Mode == dAMotorEuler
// axisMode: 0 global, 1 relative to body 1, 2 relative to body 2 (only used in dAMotorUser)
// axes are specified either globally, or relative to a body depending on mode
void BallJoint::SetAxes(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, int axisMode)
{
    dVector3 r;

    if (m_Mode == dAMotorEuler)
    {
        dJointSetAMotorMode(m_MotorJointID, dAMotorEuler);
        dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 0), x0, y0, z0, r);
        dJointSetAMotorAxis(m_MotorJointID, 0, 1, r[0], r[1], r[2]);
        dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 1), x2, y2, z2, r);
        dJointSetAMotorAxis(m_MotorJointID, 2, 2, r[0], r[1], r[2]);
    }
    else if (m_Mode == dAMotorUser)
    {
        dJointSetAMotorMode(m_MotorJointID, dAMotorUser);
        dJointSetAMotorNumAxes(m_MotorJointID, 3);
        if (axisMode == 0)
        {
            dJointSetAMotorAxis(m_MotorJointID, 0, axisMode, x0, y0, z0);
            dJointSetAMotorAxis(m_MotorJointID, 1, axisMode, x1, y1, z1);
            dJointSetAMotorAxis(m_MotorJointID, 2, axisMode, x2, y2, z2);
        }
        else
        {
            if (axisMode == 1) // relative to body1
            {
                dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 0), x0, y0, z0, r);
                dJointSetAMotorAxis(m_MotorJointID, 0, 1, r[0], r[1], r[2]);
                dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 0), x1, y1, z1, r);
                dJointSetAMotorAxis(m_MotorJointID, 0, 1, r[0], r[1], r[2]);
                dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 0), x2, y2, z2, r);
                dJointSetAMotorAxis(m_MotorJointID, 2, 2, r[0], r[1], r[2]);
            }
            else // relative to body1
            {
                dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 1), x0, y0, z0, r);
                dJointSetAMotorAxis(m_MotorJointID, 0, 1, r[0], r[1], r[2]);
                dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 1), x1, y1, z1, r);
                dJointSetAMotorAxis(m_MotorJointID, 0, 1, r[0], r[1], r[2]);
                dBodyVectorToWorld(dJointGetBody(m_MotorJointID, 1), x2, y2, z2, r);
                dJointSetAMotorAxis(m_MotorJointID, 2, 2, r[0], r[1], r[2]);
            }
        }
        // and now can set the angles based on the relative pose of the two bodies
        SetAngles();
    }

}

// this routine sets the stops for the joint
// these are relative to the axes specified in SetAxes
void BallJoint::SetStops(double a0Low, double a0High, double a1Low, double a1High, double a2Low, double a2High)
{
    if (m_Mode == dAMotorEuler || m_Mode == dAMotorUser)
    {
        // note there is safety feature that stops setting incompatible low and high
        // stops which can cause difficulties. The safe option is to set them twice.

        dJointSetAMotorParam(m_MotorJointID, dParamLoStop, a0Low);
        dJointSetAMotorParam(m_MotorJointID, dParamHiStop, a0High);
        dJointSetAMotorParam(m_MotorJointID, dParamLoStop2, a1Low);
        dJointSetAMotorParam(m_MotorJointID, dParamHiStop2, a1High);
        dJointSetAMotorParam(m_MotorJointID, dParamLoStop3, a2Low);
        dJointSetAMotorParam(m_MotorJointID, dParamHiStop3, a2High);

        dJointSetAMotorParam(m_MotorJointID, dParamLoStop, a0Low);
        dJointSetAMotorParam(m_MotorJointID, dParamHiStop, a0High);
        dJointSetAMotorParam(m_MotorJointID, dParamLoStop2, a1Low);
        dJointSetAMotorParam(m_MotorJointID, dParamHiStop2, a1High);
        dJointSetAMotorParam(m_MotorJointID, dParamLoStop3, a2Low);
        dJointSetAMotorParam(m_MotorJointID, dParamHiStop3, a2High);

        // we don't want bouncy stops
        dJointSetAMotorParam(m_MotorJointID, dParamBounce, 0);
        dJointSetAMotorParam(m_MotorJointID, dParamBounce2, 0);
        dJointSetAMotorParam(m_MotorJointID, dParamBounce3, 0);
    }

}

// calculate the angle transformation from body 1 to body 2
void BallJoint::SetAngles()
{
    if (m_Mode == dAMotorUser)
    {
        dBodyID body1 = dJointGetBody(m_MotorJointID, 0);
        dBodyID body2 = dJointGetBody(m_MotorJointID, 1);
        double thetaX, thetaY, thetaZ;

        if (body1 == 0) // body 2 is connected to the world so it is already in the correct coodinates
        {
            const double *R2 = dBodyGetRotation(body2);
            Util::EulerDecompositionXYZ(R2, thetaX, thetaY, thetaZ);
        }
        else
        {
            // find orientation of Body 2 wrt Body 1
            dMatrix3 rotMat;
            const double *R1 = dBodyGetRotation(body1);
            const double *R2 = dBodyGetRotation(body2);
            dMULTIPLY2_333(rotMat, R2, R1);

            // now find the X,Y,Z angles (use the Euler formulae for convenience not efficiency)
            Util::EulerDecompositionXYZ(rotMat, thetaX, thetaY, thetaZ);

        }

        dJointSetAMotorAngle(m_MotorJointID, 0, -thetaX);
        dJointSetAMotorAngle(m_MotorJointID, 1, -thetaY);
        dJointSetAMotorAngle(m_MotorJointID, 2, -thetaZ);
    }
}

// Get the Euler angle reference vectors
// use with care - these values are not generally altered by the user
// and are only used for state save and restore
void BallJoint::GetEulerReferenceVectors(dVector3 reference1, dVector3 reference2)
{
    dJointGetAMotorEulerReferenceVectors( m_MotorJointID , reference1 , reference2 );
}

// Set the Euler angle reference vectors
// use with care - these values are not generally altered by the user
// and are only used for state save and restore
void BallJoint::SetEulerReferenceVectors(dVector3 reference1, dVector3 reference2)
{
    dJointSetAMotorEulerReferenceVectors( m_MotorJointID , reference1 , reference2 );
}

void BallJoint::Update()
{
    if (m_Mode == dAMotorUser)
    {
        // this gets called every simulation step so it's a good place to set the angles
        SetAngles();
    }
}


void BallJoint::Dump()
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
            *m_DumpStream << "Time\tXP\tYP\tZP\ttheta0\ttheta1\ttheta2\tFX1\tFY1\tFZ1\tTX1\tTY1\tTZ1\tFX2\tFY2\tFZ2\tTX2\tTY2\tTZ2\tMotorFX1\tMotorFY1\tMotorFZ1\tMotorTX1\tMotorTY1\tMotorTZ1\tMotorFX2\tMotorFY2\tMotorFZ2\tMotorTX2\tMotorTY2\tMotorTZ2\n";
        }
    }


    if (m_DumpStream)
    {
        dVector3 p;
        GetBallAnchor(p);
        double theta0 = dJointGetAMotorAngle(m_MotorJointID, 0);
        double theta1 = dJointGetAMotorAngle(m_MotorJointID, 1);
        double theta2 = dJointGetAMotorAngle(m_MotorJointID, 2);

        *m_DumpStream << m_simulation->GetTime() << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" <<
                theta0 << "\t" << theta1 << "\t" << theta2 << "\t" <<
                m_JointFeedback.f1[0] << "\t" << m_JointFeedback.f1[1] << "\t" << m_JointFeedback.f1[2] << "\t" <<
                m_JointFeedback.t1[0] << "\t" << m_JointFeedback.t1[1] << "\t" << m_JointFeedback.t1[2] << "\t" <<
                m_JointFeedback.f2[0] << "\t" << m_JointFeedback.f2[1] << "\t" << m_JointFeedback.f2[2] << "\t" <<
                m_JointFeedback.t2[0] << "\t" << m_JointFeedback.t2[1] << "\t" << m_JointFeedback.t2[2] << "\t" <<
                m_MotorJointFeedback.f1[0] << "\t" << m_MotorJointFeedback.f1[1] << "\t" << m_MotorJointFeedback.f1[2] << "\t" <<
                m_MotorJointFeedback.t1[0] << "\t" << m_MotorJointFeedback.t1[1] << "\t" << m_MotorJointFeedback.t1[2] << "\t" <<
                m_MotorJointFeedback.f2[0] << "\t" << m_MotorJointFeedback.f2[1] << "\t" << m_MotorJointFeedback.f2[2] << "\t" <<
                m_MotorJointFeedback.t2[0] << "\t" << m_MotorJointFeedback.t2[1] << "\t" << m_MotorJointFeedback.t2[2] << "\t" <<
                "\n";
    }
}

#ifdef USE_QT
void BallJoint::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        m_physRep = new FacetedSphere(m_AxisSize[0] / 10, 4);
        m_physRep->setSimulationWindow(window);
        m_physRep->SetColour(m_Colour);
    }

    dVector3 anchor;
    dJointGetBallAnchor(m_JointID, anchor);
    m_physRep->SetDisplayPosition(anchor[0], anchor[1], anchor[2]);
    // m_PhysRep->SetDisplayRotation(r);
    m_physRep->SetColour(m_Colour);
    m_physRep->Draw();
}
#endif
