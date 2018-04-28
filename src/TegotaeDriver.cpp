/*
 *  TegotaeDriver.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 08/01/2017.
 *  Copyright 2017 Bill Sellers. All rights reserved.
 *
 */

#include "TegotaeDriver.h"

#include "Body.h"
#include "Geom.h"
#include "Contact.h"

#include <cmath>
#include <vector>
#include <algorithm>

#ifdef USE_QT
#include "SimulationWindow.h"
#include "FacetedSphere.h"
#include <QDebug>
#endif

#define RANGE(x, l, h) if (x < (l)) x = (l); if (x > (h)) x = (h);

TegotaeDriver::TegotaeDriver()
{
    m_omega   =   0;       // rad s-1     intrinsic angular velocity
    m_sigma   =   0;       // rad s-1 N-1 magnitude of sensory feedback
    m_A       =   0;       // m           positive y-direction amplitude of leg motion in a swing phase
    m_Aprime  =   0;       // m           negative y-direction amplitude of leg motion in a stance phase
    m_B       =   0;       // m           x-direction amplitude of leg motion
    m_phi     =   0;       // rad         initial phase on LF leg's oscillator

    m_X       =   0;       // m           x-direction target of leg motion (+ve forward)
    m_Y       =   0;       // m           y-direction target of leg motion (+ve further from body)
    m_N       =   0;       // N           ground reaction force
    m_phi_dot =   0;       // rad/s       the instantaneous change in phi

    m_referenceBody = 0;          // this is the body used to set the coordinate system
    m_contactGeom = 0;            // this is the geom that is used for the contact position
    m_outputVertical = false;     // this driver will output the horizontal error

    std::fill_n(m_worldGeomPosition, 3, 0);       // this is the current position of the contact GEOM in world coordinates
    std::fill_n(m_worldTargetPosition, 3, 0);   // this is the current position of the contac GEOM in reference body coordinates
    std::fill_n(m_referenceBodyGeomPosition, 3, 0);       // this is the current position of the contact GEOM in world coordinates
    std::fill_n(m_referenceBodyTargetPosition, 3, 0);   // this is the current position of the contac GEOM in reference body coordinates
    m_output = 0;                    // the output value of the controller

#ifdef USE_QT
    m_Radius = 0.1;
#endif
}

void TegotaeDriver::Initialise(double omega, double sigma, double A, double Aprime, double B, double phi,
                                              Body *referenceBody, Geom *contactGeom, const pgd::Vector &contactOffset, bool outputVertical)
{
    m_omega   =   omega;        // rad s-1     intrinsic angular velocity
    m_sigma   =   sigma;        // rad s-1 N-1 magnitude of sensory feedback
    m_A       =   A;            // m           positive y-direction amplitude of leg motion in a swing phase
    m_Aprime  =   Aprime;       // m           negative y-direction amplitude of leg motion in a stance phase
    m_B       =   B;            // m           x-direction amplitude of leg motion
    m_phi     =   phi;          // rad         initial phase on LF leg's oscillator

    m_N       =   0;            // N           ground reaction force

    m_referenceBody = referenceBody;          // this is the body used to set the coordinate system
    m_contactGeom   = contactGeom;            // this is the geom that is used for the contact position
    m_contactOffset = contactOffset;          // this is the neutral offset of the geom from the reference (m)

    m_outputVertical = outputVertical;        // this driver will output the horizontal error

    GetValue(0);                              // this sets the output values to the starting ones
}


double TegotaeDriver::GetValue(double time)
{
    if (time == m_LastTime) return m_LastValue;
    m_LastTime = time;

    double deltaT = m_simulation->GetTimeIncrement();

    // main control algorithm
    m_phi_dot = m_omega - m_sigma * m_N * std::cos(m_phi);

    // leg control
    // +ve X is relative distance forward
    // +ve Y is relative distance away from body
    // m_A is the extra distance during stance phase
    // m_Aprime is distance moved up during swing phase

    m_X = m_B * std::cos(m_phi);                   // X (0<= m_phi < 2pi)
    if (m_phi < M_PI) m_Y = m_A * std::sin(m_phi); // Y (0<= m_phi < pi)
    else m_Y = m_Aprime * std::sin(m_phi);         // Y (pi<= m_phi < 2pi)

    // get the world position of the Tegotae target
    m_referenceBodyTargetPosition[0] = m_contactOffset.x + m_X;
    m_referenceBodyTargetPosition[1] = m_contactOffset.y;
    m_referenceBodyTargetPosition[2] = m_contactOffset.z - m_Y;
    dBodyGetRelPointPos(m_referenceBody->GetBodyID(), m_referenceBodyTargetPosition[0], m_referenceBodyTargetPosition[1], m_referenceBodyTargetPosition[2], m_worldTargetPosition);

    m_contactGeom->GetWorldPosition(m_worldGeomPosition);
    dBodyGetPosRelPoint(m_referenceBody->GetBodyID(), m_worldGeomPosition[0], m_worldGeomPosition[1], m_worldGeomPosition[2], m_referenceBodyGeomPosition);

    if (m_outputVertical)
        m_output = m_worldGeomPosition[2] - m_worldTargetPosition[2]; // z direction error
    else
        m_output = m_worldGeomPosition[0] - m_worldTargetPosition[0]; // x direction error

    // update m_phi depending on m_phi_dot values
    m_phi = std::fmod(m_phi + m_phi_dot * deltaT, 2 * M_PI);
    RANGE(m_output, m_MinValue, m_MaxValue);
    m_LastValue = m_output;
    return m_output;
}

void TegotaeDriver::UpdateReactionForce()
{
    // N is the ground reaction force (GRF) acting on the leg
    m_N = 0;
    std::vector<Contact *> *contactList = m_contactGeom->GetContactList();
    dJointFeedback *jointFeedback;
    for (unsigned int i = 0; i < contactList->size(); i++)
    {
        jointFeedback = contactList->at(i)->GetJointFeedback();
        m_N += jointFeedback->f1[2];
    }
    if (m_N < 0) m_N = 0;
}


void TegotaeDriver::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (Driver::m_Name.size() == 0) std::cerr << "TegotaeDriver::Dump error: can only dump a named object\n";
            std::string filename(Driver::m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tomega\tsigma\tA\tmAprime\tB\toutputVertical"
                          << "\tX\tY\tN\tphi\tphi_dot"
                          << "\tworldGeomPositionX\tworldGeomPositionY\tworldGeomPositionZ"
                          << "\treferenceBodyGeomPositionX\treferenceBodyGeomPositionY\treferenceBodyGeomPositionZ"
                          << "\tworldTargetPositionX\tworldTargetPositionY\tworldTargetPositionZ"
                          << "\treferenceBodyTargetPositionX\treferenceBodyTargetPositionY\treferenceBodyTargetPositionZ"
                          << "\toutput"
                          << "\n";
        }
    }


    if (m_DumpStream)
    {
        *m_DumpStream << m_simulation->GetTime() << "\t" << m_omega << "\t" << m_sigma << "\t" << m_A << "\t" << m_Aprime << "\t" << m_B << "\t" << m_outputVertical
                      << "\t" << m_X << "\t" << m_Y << "\t" << m_N << "\t" << m_phi << "\t" << m_phi_dot
                      << "\t" << m_worldGeomPosition[0] << "\t" << m_worldGeomPosition[1] << "\t" << m_worldGeomPosition[2]
                      << "\t" << m_referenceBodyGeomPosition[0] << "\t" << m_referenceBodyGeomPosition[1] << "\t" << m_referenceBodyGeomPosition[2]
                      << "\t" << m_worldTargetPosition[0] << "\t" << m_worldTargetPosition[1] << "\t" << m_worldTargetPosition[2]
                      << "\t" << m_referenceBodyTargetPosition[0] << "\t" << m_referenceBodyTargetPosition[1] << "\t" << m_referenceBodyTargetPosition[2]
                      << "\t" << m_output
                      << "\n";
    }
}

double TegotaeDriver::omega() const
{
    return m_omega;
}

double TegotaeDriver::sigma() const
{
    return m_sigma;
}

double TegotaeDriver::A() const
{
    return m_A;
}

double TegotaeDriver::Aprime() const
{
    return m_Aprime;
}

double TegotaeDriver::B() const
{
    return m_B;
}

double TegotaeDriver::phi() const
{
    return m_phi;
}

double TegotaeDriver::X() const
{
    return m_X;
}

double TegotaeDriver::Y() const
{
    return m_Y;
}

double TegotaeDriver::N() const
{
    return m_N;
}


#ifdef USE_QT

void TegotaeDriver::Draw(SimulationWindow *window)
{
    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        m_physRep = new FacetedSphere(m_Radius, 4);
        m_physRep->SetColour(m_Colour);
        m_physRep->setSimulationWindow(window);
    }

//    // get the world position of the Tegotae target
//    pgd::Vector referencePosition = m_contactOffset + pgd::Vector(m_X, 0, -m_Y);
//    dVector3 worldReferencePosition;
//    dBodyGetRelPointPos(m_referenceBody->GetBodyID(), referencePosition.x, referencePosition.y, referencePosition.z, worldReferencePosition);

    // and draw the sphere
    m_physRep->SetDisplayPosition(m_worldTargetPosition[0], m_worldTargetPosition[1], m_worldTargetPosition[2]);
    m_physRep->Draw();

}

#endif

