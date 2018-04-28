/*
 *  MAMuscleExtended.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/04/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleExtended - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a serial and parallel spring element

#ifndef MAMuscleExtended_h
#define MAMuscleExtended_h

#include "Muscle.h"

class Strap;
class MAMuscle;
class DampedSpringMuscle;
class SimpleStrap;
class Filter;

class MAMuscleExtended : public Muscle
{
public:

    MAMuscleExtended(Strap *strap);
    ~MAMuscleExtended();

    void SetSerialElasticProperties(double springConstant, double unloadedLength);
    void SetParallelElasticProperties(double springConstant, double unloadedLength);
    void SetMuscleProperties(double vMax, double F0, double K);
    //void SetSmoothing(int smoothing);
    void SetActivationKinetics(bool activationKinetics) { m_ActivationKinetics = activationKinetics; }
    void SetInitialFibreLength(double initialFibreLength) { lastlpe = initialFibreLength; }

    virtual double GetMetabolicPower();

    virtual void SetActivation(double activation, double timeIncrement);
    virtual double GetActivation() { return m_Act; }
    virtual double GetElasticEnergy() { return GetESE(); }

    double GetStimulation() { return m_Stim; }

    double GetFCE() { return fce; }
    double GetLPE() { return lpe; }
    double GetFPE() { return fpe; }
    double GetLSE() { return lse; }
    double GetFSE() { return fse; }
    double GetVCE() { return vce; }
    double GetVPE() { return vce; }
    double GetVSE() { return GetVelocity() - vce; }
    double GetESE() { if (lse > sse) return (0.5 * (lse - sse) * (lse - sse) * ese); else return 0; } // energy serial element
    double GetEPE() { if (lpe > spe) return (0.5 * (lpe - spe) * (lpe - spe) * epe); else return 0; } // energy parallel element
    double GetPSE() { return GetVSE() * -fse; } // power serial element
    double GetPPE() { return GetVPE() * -fpe; } // power parallel element
    double GetPCE() { return GetVCE() * -fce; } // power contractile element
    double GetSSE() { return sse; }
    double GetSPE() { return spe; }

    virtual void Dump();
    virtual void LateInitialisation();

protected:

    double m_Stim;
    double m_Act;

    bool m_ActivationKinetics;

    double spe; // slack length parallel element (m)
    double epe; // elastic constant parallel element (N/m)
    double sse; // slack length serial element (m)
    double ese; // elastic constant serial element (N/m)

    double k; // shape constant
    double vmax; // maximum shortening velocity (m/s)
    double f0; // isometric force (N)

    double fce; // contractile force
    double lpe; // contractile and parallel length
    double fpe; // parallel element force
    double lse; // serial length
    double fse; // serial element force

    double vce; // contractile element velocity (m/s)

    double lastlpe; // last parallel element length (m)

    void CalculateForceWithParallelElement(double timeIncrement);
    void CalculateForceWithoutParallelElement(double timeIncrement);

    int m_SetActivationFirstTimeFlag;

};








#endif // MAMuscleExtended_h
