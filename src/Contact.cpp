/*
 *  Contact.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 09/02/2006.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <vector>

#include "Contact.h"
#include "PGDMath.h"
#include "FacetedPolyline.h"
#include "Simulation.h"

#ifdef USE_QT
#include "GLUtils.h"
#include "SimulationWindow.h"
#endif

// length of vector a
#define LENGTHOF(a) \
        sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

#define LENGTH2OF(a) \
        (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])


Contact::Contact()
{
#ifdef USE_QT
    m_drawContactForces = false;
    m_LastDrawTime = -1;
    m_ForceRadius = 0;
    m_ForceScale = 0;
#endif
}

Contact::~Contact()
{
//    std::cerr << "Deleting Contact\n";
}

#ifdef USE_QT
void Contact::Draw(SimulationWindow *window)
{
    const int kSides = 128;

    if (m_drawContactForces)
    {
        if (m_simulation->GetTime() != m_LastDrawTime)
        {
            m_LastDrawTime = m_simulation->GetTime();
            std::vector<FacetedObject *>::const_iterator iterFO;
            //        GLUtils::DrawAxes(m_AxisSize[0], m_AxisSize[1], m_AxisSize[2],
            //                m_ContactPosition[0], m_ContactPosition[1], m_ContactPosition[2]);

            std::vector<pgd::Vector> polyline;
            pgd::Vector startPos(m_ContactPosition[0], m_ContactPosition[1], m_ContactPosition[2]);
            pgd::Vector length(m_ContactJointFeedback.f1[0] * m_ForceScale, m_ContactJointFeedback.f1[1] * m_ForceScale, m_ContactJointFeedback.f1[2] * m_ForceScale);
            polyline.push_back(startPos);
            polyline.push_back(startPos + length);
            if (m_physRep) delete m_physRep;
            m_physRep = new FacetedPolyline(&polyline, m_ForceRadius, kSides);
            m_physRep->SetColour(m_Colour);
            m_physRep->setSimulationWindow(window);
            m_physRep->Draw();
        }
    }
}
#endif
