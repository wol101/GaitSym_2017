/*
 *  Muscle.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#ifdef USE_QT
#include "GLUtils.h"
#endif

#include "Muscle.h"

Muscle::Muscle(Strap *strap)
{
    m_Strap = strap;
#ifdef USE_QT
    m_ElasticEnergyColourFullScale = 50.0;
    m_drawMuscleForces = false;
    m_elasticDisplay = false;
#endif
}


Muscle::~Muscle()
{
    delete m_Strap;
}

#ifdef USE_QT
void Muscle::Draw(SimulationWindow *window)
{
    if (m_activationDisplay)
    {
        if (m_elasticDisplay == false)
        {
            Colour colour;
            float muscleAlpha = GetColour()->alpha;
            float muscleForceAlpha = GetForceColour()->alpha;
            GLUtils::SetColourFromMap(GetActivation(), JetColourMap, &colour, true);
            colour.alpha = muscleAlpha;
            m_Strap->SetColour(colour);
            colour.alpha = muscleForceAlpha;
            m_Strap->SetForceColour(colour);
        }
        else
        {
            Colour colour;
            float muscleAlpha = GetColour()->alpha;
            float muscleForceAlpha = GetForceColour()->alpha;
            GLUtils::SetColourFromMap(GetElasticEnergy() / m_ElasticEnergyColourFullScale, JetColourMap, &colour);
            colour.alpha = muscleAlpha;
            m_Strap->SetColour(colour);
            colour.alpha = muscleForceAlpha;
            m_Strap->SetForceColour(colour);
        }
    }
    else
    {
        m_Strap->SetColour(m_Colour);
        m_Strap->SetForceColour(m_ForceColour);
    }
    m_Strap->Draw(window);
}

bool Muscle::drawMuscleForces() const
{
    return m_drawMuscleForces;
}

void Muscle::setDrawMuscleForces(bool drawMuscleForces)
{
    m_drawMuscleForces = drawMuscleForces;
    if (m_Strap) m_Strap->setDrawMuscleForces(m_drawMuscleForces);
}

bool Muscle::activationDisplay() const
{
    return m_activationDisplay;
}

void Muscle::setActivationDisplay(bool activationDisplay)
{
    m_activationDisplay = activationDisplay;
    if (m_Strap) m_Strap->setActivationDisplay(m_activationDisplay);
}

bool Muscle::elasticDisplay() const
{
    return m_elasticDisplay;
}

void Muscle::setElasticDisplay(bool elasticDisplay)
{
    m_elasticDisplay = elasticDisplay;
}
#endif
