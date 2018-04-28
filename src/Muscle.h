/*
 *  Muscle.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Muscle_h
#define Muscle_h

#include "Drivable.h"
#include "Strap.h"
#include "Simulation.h"
#include <vector>
#include <string>

class Muscle:public Drivable
{
public:

    Muscle(Strap *strap);
    virtual ~Muscle();

    double GetLength() { return m_Strap->GetLength(); }
    double GetVelocity() { return m_Strap->GetVelocity(); }
    double GetTension() { return m_Strap->GetTension(); }
    double GetPower() { return -m_Strap->GetTension() * m_Strap->GetVelocity(); }

    void CalculateStrap(double deltaT) { m_Strap->Calculate(deltaT); }

    virtual void SetActivation(double activation, double  duration) = 0;
    virtual double GetActivation() = 0;
    virtual double GetMetabolicPower() = 0;
    virtual double GetElasticEnergy() = 0;

    std::vector<PointForce *> *GetPointForceList() { return m_Strap->GetPointForceList(); }

    Strap *GetStrap() { return m_Strap; }

    void SetAllVisible(bool v) { SetVisible(v);  m_Strap->SetVisible(v); }

    virtual int SanityCheck(Muscle *otherMuscle, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight)
    {
        return m_Strap->SanityCheck(otherMuscle->m_Strap, axis, sanityCheckLeft, sanityCheckRight);
    }

    virtual void LateInitialisation() { CalculateStrap(0); m_Strap->SetName(m_Name + std::string("Strap")); }

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
    void SetForceColour(const Colour &colour) { m_ForceColour = colour; }
    const Colour *GetForceColour() { return &m_ForceColour; }

    bool drawMuscleForces() const;
    void setDrawMuscleForces(bool drawMuscleForces);

    bool activationDisplay() const;
    void setActivationDisplay(bool activationDisplay);

    bool elasticDisplay() const;
    void setElasticDisplay(bool elasticDisplay);

#endif

protected:

    Strap *m_Strap;

#ifdef USE_QT
    Colour m_ForceColour;
    double m_ElasticEnergyColourFullScale; // this value represents full scale for the elastic energy colour
    bool m_drawMuscleForces;
    bool m_activationDisplay;
    bool m_elasticDisplay;
#endif

};

#endif

