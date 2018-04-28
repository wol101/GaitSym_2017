/*
 *  MAMuscleComplete.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 03/03/2007.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleComplete - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a serial and parallel spring element
// plus length tension stuff too


#ifndef MAMuscleComplete_h
#define MAMuscleComplete_h

#include "Muscle.h"

class Strap;
class MAMuscle;
class DampedSpringMuscle;
class SimpleStrap;
class Filter;

class MAMuscleComplete : public Muscle
{
public:
    enum StrainModel
    {
        linear = 0,
        square
    };

    // this struct contains all the paramers required for the CalculateForceError function
    struct CalculateForceErrorParams
    {
        // fixed input parameters
        double spe; // slack length parallel element (m)
        double epe; // elastic constant parallel element (N/m)
        double dpe; // damping constant parallel element (N/m)
        StrainModel smpe; // strain model for parallel element
        double sse; // slack length serial element (m)
        double ese; // elastic constant serial element (N/m)
        double dse; // damping constant serial element (N/m)
        StrainModel smse; // strain model for serial element
        double k; // shape constant
        double vmax; // maximum shortening velocity (m/s)
        double fmax; // maximum isometric force (N)
        double width; // relative width of length/tension peak

        // variable input parameters
        double alpha; // proportion of muscle activated
        double timeIncrement; // inegration step size for simulation (s)
        double len; // length of the whole element (m)
        double v; // contraction velocity of the whole element (m/s)
        double lastlpe; // last calculated lpe value (m)

        // output parameters
        double fce; // contractile force (N)
        double lpe; // contractile and parallel length (m)
        double fpe; // parallel element force (N)
        double lse; // serial length (m)
        double fse; // serial element force (N)
        double vce; // contractile element velocity (m/s)
        double vse; // serial element velocity (m/s)
        double targetFce; // fce calculated from elastic elements (N)
        double f0; // length corrected fmax (N)
        double err; // error term in lpe (m)
    };

    MAMuscleComplete(Strap *strap);
    ~MAMuscleComplete();

    void SetSerialElasticProperties(double serialStrainAtFmax, double serialStrainRateAtFmax, double tendonLength, MAMuscleComplete::StrainModel serialStrainModel);
    void SetParallelElasticProperties(double parallelStrainAtFmax, double serialStrainRateAtFmax, double parallelElementLength, MAMuscleComplete::StrainModel parallelStrainModel);
    void SetMuscleProperties(double vMax, double Fmax, double K, double Width);
    void SetActivationKinetics(bool activationKinetics, double akFastTwitchProportion, double akTActivationA, double akTActivationB, double akTDeactivationA, double akTDeactivationB);
    void SetInitialFibreLength(double initialFibreLength) { m_Params.lastlpe = initialFibreLength; }
    void SetActivationRate(double activationRate) { m_ActivationRate = activationRate; }
    void SetStartActivation(double startActivation) { m_Params.alpha = startActivation; }
    void SetMinimumActivation(double minimumActivation) { m_MinimumActivation = minimumActivation; }

    virtual double GetMetabolicPower();

    virtual void SetActivation(double activation, double timeIncrement);
    virtual double GetActivation() { return m_Params.alpha; }
    virtual double GetElasticEnergy() { return GetESE(); }

    double GetStimulation() { return m_Stim; };

    double GetFCE() { return m_Params.fce; } // contractile force (N)
    double GetLPE() { return m_Params.lpe; } // contractile and parallel length (m)
    double GetFPE() { return m_Params.fpe; } // parallel element force (N)
    double GetLSE() { return m_Params.lse; } // serial length (m)
    double GetFSE() { return m_Params.fse; } // serial element force (N)
    double GetVCE() { return m_Params.vce; } // contractile element velocity (m/s)
    double GetVPE() { return m_Params.vce; } // parallel element velocity (m/s)
    double GetVSE() { return m_Params.vse; } // serial element velocity (m/s)
    double GetESE(); // energy serial element
    double GetEPE(); // energy parallel element
    double GetPSE() { return GetVSE() * -m_Params.fse; } // power serial element
    double GetPPE() { return GetVPE() * -m_Params.fpe; } // power parallel element
    double GetPCE() { return GetVCE() * -m_Params.fce; } // power contractile element
    double GetSSE() { return m_Params.sse; }
    double GetSPE() { return m_Params.spe; }

    virtual void Dump();
    virtual void LateInitialisation();

protected:

    double m_Stim;
    bool m_ActivationKinetics;
    double m_ft;
    double m_tact;
    double m_tdeact;
    double m_ActivationRate;
    double m_serialStrainAtFmax;
    double m_serialStrainRateAtFmax;
    double m_parallelStrainAtFmax;
    double m_parallelStrainRateAtFmax;
    double m_MinimumActivation;

    CalculateForceErrorParams m_Params;
    double m_Tolerance;
    int m_MaxIter;

    int m_SetActivationFirstTimeFlag;

};






#endif // MAMuscleComplete_h
