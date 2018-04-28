/*
 *  Strap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "Strap.h"
#include "Body.h"
#include "Simulation.h"

#ifdef USE_QT
#include "FacetedObject.h"
#endif

Strap::Strap()
{
    m_Length = 0;
    m_LastLength = 0;
    m_Velocity = 0;
    m_Tension = 0;

#ifdef USE_QT
    m_ForceColour.SetColour(1, 1, 1, 1);
    m_ForceRadius = 0;
    m_ForceScale = 0;
    m_Radius = 0;
    m_LastDrawTime = -1;
    m_drawMuscleForces = false;
    m_activationDisplay = false;
#endif

}

Strap::~Strap()
{
    std::vector<PointForce *>::const_iterator iter1;
    for (iter1 = m_PointForceList.begin(); iter1 != m_PointForceList.end(); iter1++)
        delete *iter1;

#ifdef USE_QT
    std::vector<FacetedObject *>::const_iterator iterFO;
    for (iterFO = m_DrawList.begin(); iterFO != m_DrawList.end(); iterFO++)
        delete *iterFO;
#endif
}

void Strap::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "Strap::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tBody\tXP\tYP\tZP\tFX\tFY\tFZ\n";
        }
    }


    if (m_DumpStream)
    {
        std::vector<PointForce *>::const_iterator iter1;
        for (iter1 = m_PointForceList.begin(); iter1 != m_PointForceList.end(); iter1++)
        {
            *m_DumpStream << m_simulation->GetTime() << "\t" << *((*iter1)->body->GetName()) << "\t" <<
                             (*iter1)->point[0] << "\t" << (*iter1)->point[1] << "\t" << (*iter1)->point[2] << "\t" <<
                             (*iter1)->vector[0] * m_Tension << "\t" << (*iter1)->vector[1] * m_Tension << "\t" << (*iter1)->vector[2] * m_Tension << "\n";
        }
    }
}

#ifdef USE_QT
bool Strap::drawMuscleForces() const
{
    return m_drawMuscleForces;
}

void Strap::setDrawMuscleForces(bool drawMuscleForces)
{
    m_drawMuscleForces = drawMuscleForces;
}

bool Strap::activationDisplay() const
{
    return m_activationDisplay;
}

void Strap::setActivationDisplay(bool activationDisplay)
{
    m_activationDisplay = activationDisplay;
}
#endif

