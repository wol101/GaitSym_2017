/*
 *  DampedSpringMuscle.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// DampedSpringMuscle - implementation of a damped spring strap force

#include <ode/ode.h>

#include "Strap.h"
#include "DampedSpringMuscle.h"
#include "DebugControl.h"
#include "Simulation.h"

// constructor

DampedSpringMuscle::DampedSpringMuscle(Strap *strap): Muscle(strap)
{
    m_Damping = 0;
    m_SpringConstant = 0;
    m_UnloadedLength = 0;
    m_Activation = 0;
    m_Area = 1;
}

// destructor
DampedSpringMuscle::~DampedSpringMuscle()
{
}

double DampedSpringMuscle::GetElasticEnergy()
{
    double delLen = m_Strap->GetLength() - m_UnloadedLength;
    if (delLen < 0) return 0;

    // difference between these two values is the amount of energy lost by damping
    // std::cerr << 0.5 * m_Strap->GetTension() * delLen << "\n";
    // std::cerr << 0.5 * m_SpringConstant * m_Area * delLen * delLen / m_UnloadedLength << "\n";

    return 0.5 * m_SpringConstant * m_Area * delLen * delLen / m_UnloadedLength;
}


// update the tension depending on length and velocity
// activation is used as a linear multiplier
void DampedSpringMuscle::SetActivation(double activation, double /* duration */)
{
    m_Activation = activation;

    // calculate strain
    double elasticStrain = (m_Strap->GetLength() - m_UnloadedLength) / m_UnloadedLength;

    // calculate stress
    double elasticStress = elasticStrain * m_SpringConstant;
    double tension;
    if (elasticStress <= 0) // if not stretching the spring then set tension to zero
    {
        tension = 0;
    }
    else
    {

        // calculate damping (+ve when lengthening)
        double relativeVelocity = m_Strap->GetVelocity() / m_UnloadedLength;
        double dampingStress = relativeVelocity * m_Damping;

        // now calculate tension
        // NB. tension is negative when muscle shortening
        tension = (elasticStress + dampingStress) * m_Area * m_Activation;

        // stop any pushing
        if (tension < 0) tension = 0;
    }
    m_Strap->SetTension(tension);

    if (gDebug == DampedSpringDebug)
        *gDebugStream << "DampedSpringMuscle::UpdateTension m_Name " << m_Name << " m_Strap->GetLength() " << m_Strap->GetLength() << " m_UnloadedLength " << m_UnloadedLength
            << " m_SpringConstant " << m_SpringConstant << " m_Strap->GetVelocity() " << m_Strap->GetVelocity()
            << " m_Damping " << m_Damping << " tension " << tension << "\n";
}

void DampedSpringMuscle::Dump()
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
            *m_DumpStream << "Time\tact\ttension\tlength\tvelocity\tPMECH\n";
        }
    }


    if (m_DumpStream)
    {
        *m_DumpStream << m_simulation->GetTime() << "\t" << m_Activation <<
                "\t" << m_Strap->GetTension() << "\t" << m_Strap->GetLength() << "\t" << m_Strap->GetVelocity() <<
                "\t" << m_Strap->GetVelocity() * m_Strap->GetTension() <<
                "\n";
    }
}


