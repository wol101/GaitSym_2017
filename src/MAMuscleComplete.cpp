/*
 *  MAMuscleComplete.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 03/03/2007.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleComplete - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow damped spring element
// plus length tension stuff too

#define SQUARE(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

#include "Strap.h"
#include "MAMuscleComplete.h"
#include "DebugControl.h"
#include "Simulation.h"

#include <cmath>

static double CalculateForceError (double lce, void *params);
static double zeroin(double *ax, double *bx, double (*f)(double x, void *info), void *info, double *tol);

// constructor

MAMuscleComplete::MAMuscleComplete(Strap *strap): Muscle(strap)
{
    m_Stim = 0;
    m_ActivationKinetics = false;
    m_ft = 0;
    m_tact = 0;
    m_tdeact = 0;
    m_ActivationRate = 0;
    m_serialStrainAtFmax = 0;
    m_serialStrainRateAtFmax = 0;
    m_parallelStrainAtFmax = 0;
    m_parallelStrainRateAtFmax = 0;
    m_MinimumActivation = 0.001; // arbitrary value so that we avoid some numerical issues


    m_Params.spe = 0; // slack length parallel element (m)
    m_Params.epe = 0; // elastic constant parallel element (N/m)
    m_Params.dpe = 0; // damping constant parallel element (Ns/m)
    m_Params.smpe = linear; // strain model for parallel element
    m_Params.sse = 0; // slack length serial element (m)
    m_Params.ese = 0; // elastic constant serial element (N/m)
    m_Params.dse = 0; // damping constant serial element (Ns/m)
    m_Params.smse = linear; // strain model for serial element
    m_Params.k = 0; // shape constant
    m_Params.vmax = 0; // maximum shortening velocity (m/s)
    m_Params.fmax = 0; // maximum isometric force (N)
    m_Params.width = 0; // relative width of length/tension peak

    m_Params.alpha = 0; // proportion of muscle activated
    m_Params.timeIncrement = 0; // inegration step size for simulation
    m_Params.len = 0; // length of the whole element (m)
    m_Params.lastlpe = -1; // last calculated lpe value

    // output parameters set to dummy value
    m_Params.fce = -2; // contractile force (N)
    m_Params.lpe = -2; // contractile and parallel length (m)
    m_Params.fpe = -2; // parallel element force (N)
    m_Params.lse = -2; // serial length (m)
    m_Params.fse = -2; // serial element force (N)
    m_Params.vce = -2; // contractile element velocity (m/s)
    m_Params.vse = -2; // serial element velocity (m/s)
    m_Params.targetFce = -2; // fce calculated from elastic elements (N)
    m_Params.f0 = -2; // length corrected fmax (N)
    m_Params.err = -2; // error term in lpe (m)

    m_Tolerance = 1e-8; // solution tolerance (m) - small because the serial tendons are quite stiff
    m_MaxIter = 100; // max iterations to find solution

    m_SetActivationFirstTimeFlag = true;

}

// destructor
MAMuscleComplete::~MAMuscleComplete()
{
}

// set the muscle elastic properties
void MAMuscleComplete::SetSerialElasticProperties(double serialStrainAtFmax, double serialStrainRateAtFmax, double tendonLength, MAMuscleComplete::StrainModel serialStrainModel)
{
    double serialDampingConstant, serialElasticConstant;
    m_serialStrainAtFmax = serialStrainAtFmax;
    m_serialStrainRateAtFmax = serialStrainRateAtFmax;
    m_Params.smse = serialStrainModel; // strain model for serial element
    m_Params.sse = tendonLength; // slack length serial element (m)

    if (tendonLength != -1) // if tendonLength == -1 this has to be done later once the tendonLength can be calculated
    {
        switch (serialStrainModel)
        {
        case MAMuscleComplete::linear:
            serialElasticConstant = m_Params.fmax/(serialStrainAtFmax * tendonLength);
            break;
        case MAMuscleComplete::square:
            serialElasticConstant = m_Params.fmax/(SQUARE(serialStrainAtFmax * tendonLength));
            break;
        }
        if (serialStrainRateAtFmax == 0) serialDampingConstant = 0;
        else serialDampingConstant = m_Params.fmax/(serialStrainRateAtFmax * tendonLength);
        m_Params.ese = serialElasticConstant; // elastic constant serial element (N/m)
        m_Params.dse = serialDampingConstant; // damping constant serial element (Ns/m)
    }
}

void MAMuscleComplete::SetParallelElasticProperties(double parallelStrainAtFmax, double parallelStrainRateAtFmax, double parallelElementLength, MAMuscleComplete::StrainModel parallelStrainModel)
{
    double parallelDampingConstant, parallelElasticConstant;

    m_parallelStrainAtFmax = parallelStrainAtFmax;
    m_parallelStrainRateAtFmax = parallelStrainRateAtFmax;

    if (parallelStrainAtFmax == 0) parallelElasticConstant = 0;
    else
    {
        switch (parallelStrainModel)
        {
        case MAMuscleComplete::linear:
            parallelElasticConstant = m_Params.fmax/(parallelStrainAtFmax * parallelElementLength);
            break;
        case MAMuscleComplete::square:
            parallelElasticConstant = m_Params.fmax/(SQUARE(parallelStrainAtFmax * parallelElementLength));
            break;
        }
    }
    if (parallelStrainRateAtFmax == 0) parallelDampingConstant = 0;
    else parallelDampingConstant = m_Params.fmax/(parallelStrainRateAtFmax * parallelElementLength);

    m_Params.spe = parallelElementLength; // slack length parallel element (m)
    m_Params.epe = parallelElasticConstant; // elastic constant parallel element (N/m)
    m_Params.dpe = parallelDampingConstant; // damping constant serial element (Ns/m)
    m_Params.smpe = parallelStrainModel; // strain model for serial element
}

// set the muscle contractile properties
void MAMuscleComplete::SetMuscleProperties(double vMax, double Fmax, double K, double Width)
{
    m_Params.k = K; // shape constant
    m_Params.vmax = vMax; // maximum shortening velocity (m/s)
    m_Params.fmax = Fmax; // isometric force
    m_Params.width = Width; // relative width of length/tension peak
}

// set the activation kinetics
void MAMuscleComplete::SetActivationKinetics(bool activationKinetics,
                                             double akFastTwitchProportion,
                                             double akTActivationA, double akTActivationB,
                                             double akTDeactivationA, double akTDeactivationB)
{
    m_ActivationKinetics = activationKinetics;

    // original code
    //double ft = 0.5; // arbitrary set activation kinetics as 50% fast twitch
    //double tact = 80e-3 - 0.47e-3 * ft; // Umberger et al 2003 eq 4
    //double tdeact = 90e-3 - 0.56e-3 * ft; // Umberger et al 2003 eq 4

    m_ft = akFastTwitchProportion;
    m_tact = akTActivationA + akTActivationB * m_ft;
    m_tdeact = akTDeactivationA + akTDeactivationB * m_ft;
}

// do any intialisation that relies on the strap being set up properly

void MAMuscleComplete::LateInitialisation()
{
    Muscle::LateInitialisation();
    m_Params.len = m_Strap->GetLength();

    // lastlpe perhaps not set to anything useful

    // handle sse == -1 which means I need to calculate this value internally and reset the derived values
    if (m_Params.sse == -1)
    {
        m_Params.sse = m_Params.len - m_Params.spe; // set the slack serial length

        double serialDampingConstant, serialElasticConstant;
        switch (m_Params.smse)
        {
        case MAMuscleComplete::linear:
            serialElasticConstant = m_Params.fmax/(m_serialStrainAtFmax * m_Params.sse);
            break;
        case MAMuscleComplete::square:
            serialElasticConstant = m_Params.fmax/(SQUARE(m_serialStrainAtFmax * m_Params.sse));
            break;
        }
        if (m_serialStrainRateAtFmax == 0) serialDampingConstant = 0;
        else serialDampingConstant = m_Params.fmax/(m_serialStrainRateAtFmax * m_Params.sse);
        m_Params.ese = serialElasticConstant; // elastic constant serial element (N/m)
        m_Params.dse = serialDampingConstant; // damping constant serial element (Ns/m)
    }

    double minlpe = m_Params.spe - (m_Params.spe * m_Params.width / 2);
    if (minlpe < 0) minlpe = 0;
    double maxlpe = m_Params.len - m_Params.sse; // this would be right with no damping
    if (maxlpe < minlpe) maxlpe = minlpe;
    if (m_Params.lastlpe == -1)
    {
        // just setting lastlpe to something sensible should be enough
        m_Params.lastlpe = (maxlpe + minlpe) / 2;
    }
    m_Params.lpe = m_Params.lastlpe; // needed so that we get a sensible t=0 OutputProgramState

    return;

}

// set the proportion of muscle fibres that are active
// calculates the tension in the strap

void MAMuscleComplete::SetActivation(double activation, double timeIncrement)
{
/*
    if (m_Name == "RightSoleus")
    {
        if (m_simulation->GetTime() > 0.12805)
        {
            std::cerr << "Reached debug point\n";
        }
    }
*/
    // set variable input parameters

    if (activation < m_MinimumActivation) activation = m_MinimumActivation;
    else if (activation > 1) activation = 1;
    m_Stim = activation;

    m_Params.timeIncrement = timeIncrement;
    if (m_ActivationKinetics || m_ActivationRate != 0)
    {
        if (m_Params.alpha == -1) // special case for first run through if I just want disable rate
        {
            m_Params.alpha = m_Stim;
        }
        else
        {
            if (m_ActivationKinetics)
            {
                // using activation kinetics from UGM model
                double t2 = 1 / m_tdeact;
                double t1 = 1 / m_tact - t2;
                // Nagano & Gerritsen 2001 A2
                double qdot = (m_Stim - m_Params.alpha) * (t1 * m_Stim + t2);
                m_Params.alpha += qdot * m_Params.timeIncrement;
                // I think I should allow this to fall - it won't make any difference and it maintains
                // continuity in the differentials
                // if (m_Params.alpha < 0.001) m_Params.alpha = 0.001; // m_Act never drops to zero in practice
            }
            else // this is if the activation rate is limited
            {
                double delAct = m_ActivationRate * m_Params.timeIncrement;
                if (m_Stim > m_Params.alpha)
                {
                    m_Params.alpha += delAct;
                    if (m_Params.alpha > m_Stim) m_Params.alpha = m_Stim;
                }
                else if (m_Stim < m_Params.alpha)
                {
                    m_Params.alpha -= delAct;
                    if (m_Params.alpha < m_Stim) m_Params.alpha = m_Stim;
                }
            }
        }
    }
    else
    {
        m_Params.alpha = m_Stim;
    }

    m_Params.len = m_Strap->GetLength();
    m_Params.v = m_Strap->GetVelocity();

    double minlpe = m_Params.spe - (m_Params.spe * m_Params.width / 2);
    if (minlpe < 0) minlpe = 0;
    // double maxlpe = m_Params.len - m_Params.sse; // this would be right with no damping
    // if (maxlpe < minlpe) maxlpe = minlpe;

    // now calculate output parameters

    // need to do some checks here for being slack (and also silly extension/contraction rates???)
    double minlen;
    minlen = m_Params.sse + minlpe;
    if (m_Params.len <= minlen)
    {
        m_Params.lastlpe = minlpe;

        m_Params.fce = 0; // contractile force (N)
        m_Params.lpe = minlpe; // contractile and parallel length (m)
        m_Params.fpe = 0; // parallel element force (N)
        m_Params.lse = m_Params.sse; // serial length (m)
        m_Params.fse = 0; // serial element force (N)
        m_Params.vce = 0; // contractile element velocity (m/s)
        m_Params.vse = 0; // serial element velocity (m/s)
        m_Params.targetFce = 0; // fce calculated from elastic elements (N)
        m_Params.f0 = 0; // length corrected fmax (N)
        m_Params.err = 0; // error term in lpe (m)
    }
    else
    {
        if (m_Params.ese == 0) // special case - easy to calculate
        {
            m_Params.err = CalculateForceError (m_Params.len - m_Params.sse, &m_Params);
            m_Params.lastlpe = m_Params.lpe;
        }
        else
        {

            // now solve the activation function so the contractile and elastic elements are consistent

            // we have a previous value for lce that is probably a good estimate of the new values
            double currentEstimate = m_Params.lastlpe;
            if (currentEstimate < 0) currentEstimate = 0;
            if (currentEstimate > m_Params.len) currentEstimate = m_Params.len;
            double flast = CalculateForceError(currentEstimate, &m_Params);
            if (fabs(flast) <= m_Tolerance)
            {
                m_Params.err = flast;
            }
            else
            {
                double ax, bx, r, tol;
                // double range = maxlpe - minlpe; // this doesn't quite work because of damping
                double range = m_Params.len; // this should be bigger than necessary
                int nInc = 100;
                double inc = range / nInc;
                double high_target, low_target, err;
                int i;
                for (i = 1; i <= nInc; i++)
                {
                    high_target = currentEstimate + i * inc;
                    low_target = currentEstimate - i * inc;
                    if (high_target > m_Params.len) high_target = m_Params.len;
                    if (low_target < 0) low_target = 0;

                    if (high_target <= m_Params.len) // maxlpe might be expected to work but is too small
                    {
                        err = CalculateForceError(high_target, &m_Params);
                        if (std::signbit(err) != std::signbit(flast))
                        {
                            ax = currentEstimate + (i - 1) * inc;
                            bx = high_target;
                            break;
                        }
                    }
                    if (low_target >= 0) // minlpe might be expected to work but is too big
                    {
                        err = CalculateForceError(low_target, &m_Params);
                        if (std::signbit(err) != std::signbit(flast))
                        {
                            ax = currentEstimate - (i - 1) * inc;
                            bx = low_target;
                            break;
                        }
                    }
                    if (high_target >= m_Params.len && low_target <= 0) i = nInc + 1;
                }
                if (i > nInc)
                {
                    std::cerr << "MAMuscleComplete::SetActivation Error: Unable to solve lpe " << m_Name << "\n";
                    m_Params.err = CalculateForceError (currentEstimate, &m_Params); // couldn't find anything better
                    m_Params.lastlpe = currentEstimate;
                }
                else
                {
                    tol = m_Tolerance;
                    r = zeroin(&ax, &bx, &CalculateForceError, &m_Params, &tol);
                    m_Params.err = CalculateForceError (r, &m_Params); // this sets m_Params with all the correct values
                    m_Params.lastlpe = r;
                }
            }
        }
    }

    m_Strap->SetTension(m_Params.fse);

    if (gDebug == MAMuscleCompleteDebug)
    {
        if (DebugFilters("SetActivation", m_Name))
        {
            if (m_SetActivationFirstTimeFlag) // only happens once
            {
                m_SetActivationFirstTimeFlag = false;
                *gDebugStream << "MAMuscleComplete::SetActivation " <<
                                 " " << "spe" <<  // slack length parallel element (m)
                                 " " << "epe" <<  // elastic constant parallel element (N/m)
                                 " " << "dpe" <<  // damping constant parallel element (N/m)
                                 " " << "smpe" << // strain model for parallel element
                                 " " << "sse" <<  // slack length serial element (m)
                                 " " << "ese" <<  // elastic constant serial element (N/m)
                                 " " << "dse" <<  // damping constant serial element (N/m)
                                 " " << "smse" << // strain model for serial element
                                 " " << "k" <<  // shape constant
                                 " " << "vmax" <<  // maximum shortening velocity (m/s)
                                 " " << "fmax" <<  // maximum isometric force (N)
                                 " " << "width" <<  // relative width of length/tension peak

                                 // variable input parameters
                                 " " << "alpha" <<  // proportion of muscle activated
                                 " " << "timeIncrement" <<  // inegration step size for simulation (s)
                                 " " << "len" <<  // length of the whole element (m)
                                 " " << "v" <<  // contraction velocity of the whole element (m/s)
                                 " " << "lastlpe" <<  // last calculated lpe value (m)

                                 // output parameters
                                 " " << "fce" <<  // contractile force (N)
                                 " " << "lpe" <<  // contractile and parallel length (m)
                                 " " << "fpe" <<  // parallel element force (N)
                                 " " << "lse" <<  // serial length (m)
                                 " " << "fse" <<  // serial element force (N)
                                 " " << "vce" <<  // contractile element velocity (m/s)
                                 " " << "vse" <<  // serial element velocity (m/s)
                                 " " << "targetFce" <<  // fce calculated from elastic elements (N)
                                 " " << "f0" <<  // length corrected fmax (N)
                                 " " << "err" <<  // error term in lpe (m)

                                 "\n";
            }

            *gDebugStream << m_Name <<
                             " " << m_Params.spe <<  // slack length parallel element (m)
                             " " << m_Params.epe <<  // elastic constant parallel element (N/m)
                             " " << m_Params.dpe <<  // damping constant parallel element (N/m)
                             " " << m_Params.smpe << // strain model for parallel element
                             " " << m_Params.sse <<  // slack length serial element (m)
                             " " << m_Params.ese <<  // elastic constant serial element (N/m)
                             " " << m_Params.dse <<  // damping constant serial element (N/m)
                             " " << m_Params.smse << // strain model for serial element
                             " " << m_Params.k <<  // shape constant
                             " " << m_Params.vmax <<  // maximum shortening velocity (m/s)
                             " " << m_Params.fmax <<  // maximum isometric force (N)
                             " " << m_Params.width <<  // relative width of length/tension peak

                             // variable input parameters
                             " " << m_Params.alpha <<  // proportion of muscle activated
                             " " << m_Params.timeIncrement <<  // inegration step size for simulation (s)
                             " " << m_Params.len <<  // length of the whole element (m)
                             " " << m_Params.v <<  // contraction velocity of the whole element (m/s)
                             " " << m_Params.lastlpe <<  // last calculated lpe value (m)

                             // output parameters
                             " " << m_Params.fce <<  // contractile force (N)
                             " " << m_Params.lpe <<  // contractile and parallel length (m)
                             " " << m_Params.fpe <<  // parallel element force (N)
                             " " << m_Params.lse <<  // serial length (m)
                             " " << m_Params.fse <<  // serial element force (N)
                             " " << m_Params.vce <<  // contractile element velocity (m/s)
                             " " << m_Params.vse <<  // serial element velocity (m/s)
                             " " << m_Params.targetFce <<  // fce calculated from elastic elements (N)
                             " " << m_Params.f0 <<  // length corrected fmax (N)
                             " " << m_Params.err <<  // error term in lpe (m)
                             "\n";
        }
    }
}

// calculate the metabolic power of the muscle

double MAMuscleComplete::GetMetabolicPower()
{
    // m_Velocity is negative when muscle shortening
    // we need the sign the other way round
    double relV = -m_Params.vce / m_Params.vmax;

    // limit relV
    if (relV > 1) relV = 1;
    else if (relV < -1) relV = -1;

    double relVSquared = relV * relV;
    double relVCubed = relVSquared * relV;

    double sigma = (0.054 + 0.506 * relV + 2.46 * relVSquared) /
        (1 - 1.13 * relV + 12.8 * relVSquared - 1.64 * relVCubed);

    double power = m_Params.alpha * m_Params.f0 * m_Params.vmax * sigma;

    if (gDebug == MAMuscleCompleteDebug)
    {
        if (DebugFilters("GetMetabolicPower", m_Name))
        {
            *gDebugStream << "MAMuscle::GetMetabolicPower " << m_Name <<
            " alpha " << m_Params.alpha <<
            " f0 " << m_Params.f0 <<
            " vmax " << m_Params.vmax <<
            " m_Velocity " << m_Strap->GetVelocity() <<
            " sigma " << sigma <<
            " power " << power << "\n";
        }
    }
    return (power);
}

// note that with damping this is the energy stored but not the energy that will be returned
double MAMuscleComplete::GetESE() // energy serial element
{
    if (m_Params.lse <= m_Params.sse) return 0;

    double energy;
    double extension = m_Params.lse - m_Params.sse;
    // serial element model
    switch (m_Params.smpe)
    {
    case MAMuscleComplete::linear:
        energy = (0.5 * SQUARE(extension) * m_Params.ese);
        break;

    case MAMuscleComplete::square:
        energy = ((1.0/3.0) * CUBE(extension) * m_Params.ese);
        break;
    }

    return energy;
}

// note that with damping this is the energy stored but not the energy that will be returned
double MAMuscleComplete::GetEPE() // energy serial element
{
    if (m_Params.lpe <= m_Params.spe) return 0;

    double energy;
    double extension = m_Params.lpe - m_Params.spe;
    // serial element model
    switch (m_Params.smpe)
    {
    case MAMuscleComplete::linear:
        energy = (0.5 * SQUARE(extension) * m_Params.epe);
        break;

    case MAMuscleComplete::square:
        energy = ((1.0/3.0) * CUBE(extension) * m_Params.epe);
        break;
    }

    return energy;
}

// we can calculate the force from the length of the contractile element in two independent ways:
// (1) from the elastic properties of the serial and elastic elements
// (2) from the contractile element physiology and its activation
// this function calculates the error for the force as calculated by the elastic elements and the contractile elements
// this value should be zero for everything to work as expected and that's what the solver will attempt to achieve
double CalculateForceError (double lce, void *params)
{
    MAMuscleComplete::CalculateForceErrorParams *p = (MAMuscleComplete::CalculateForceErrorParams *)params;

    // The elastic elements each generate a force and fce = fse - fpe

    p->lpe = lce;
    p->lse = p->len - p->lpe;
    p->vce = (p->lpe - p->lastlpe) / p->timeIncrement;
    p->vse = p->v - p->vce;

    // parallel element
    if (p->lpe <= p->spe) p->fpe = 0;
    else
    {
        switch (p->smpe)
        {
        case MAMuscleComplete::linear:
            p->fpe = p->epe * (p->lpe - p->spe) + p->dpe * p->vce; // positive sign because force gets higher as speed of extension increases
            break;

        case MAMuscleComplete::square:
            p->fpe = p->epe * SQUARE(p->lpe - p->spe) + p->dpe * p->vce; // positive sign because force gets higher as speed of extension increases
            break;
        }
        if (p->fpe < 0) p->fpe = 0;
    }


    // serial element
    if (p->lse <= p->sse) p->fse = 0;
    else
    {
        switch (p->smpe)
        {
        case MAMuscleComplete::linear:
            p->fse = p->ese * (p->lse - p->sse) + p->dse * p->vse; // positive sign because force gets higher as speed of extension increases
            break;

        case MAMuscleComplete::square:
            p->fse = p->ese * SQUARE(p->lse - p->sse) + p->dse * p->vse; // positive sign because force gets higher as speed of extension increases
            break;
        }
        if (p->fse < 0) p->fse = 0;
    }


    // can calculate the fce based on the elastic elements
    p->targetFce = p->fse - p->fpe;

    // contractile element based fce calculate

    // use the width to calculate the maximum force for the ce
    p->f0 = p->fmax * (1 - (4 * SQUARE(-1 + p->lpe/p->spe))/p->width);
    if (p->f0 <= 0)
    {
        p->f0 = 0;
        p->fce = 0;
    }
    else
    {
        if (p->alpha == 0)
        {
            p->fce = 0;
        }
        else
        {
            double localvce = p->vce;
            if (localvce > p->vmax) localvce = p->vmax; // velocity sanity limits
            if (localvce < -p->vmax) localvce = -p->vmax; // velocity sanity limits

            if (localvce > 0) // eccentric
            {
                p->fce = p->alpha * p->f0 * (1.8 + (0.8 * p->k*(localvce - 1.0 * p->vmax)) / (7.56 * localvce + p->k * p->vmax));
            }
            else // concentric
            {
                p->fce = (p->alpha * p->f0 * p->k * (localvce + p->vmax)) / (-localvce + p->k * p->vmax);
            }

        }
    }

    double err = p->fce - p->targetFce;
    // std::cerr << "lce = " << lce << " Error = " << err << "\n";
    return err;

}

void MAMuscleComplete::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "MAMuscleComplete::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tm_Stim\talpha\tlen\tv\tlastlpe\tfce\tlpe\tfpe\tlse\tfse\tvce\tvse\ttargetFce\tf0\terr\tESE\tEPE\tPSE\tPPE\tPCE\ttension\tlength\tvelocity\tPMECH\tPMET\n";
        }
    }


    if (m_DumpStream)
    {
        *m_DumpStream << m_simulation->GetTime() << "\t" <<
                         m_Stim << "\t" << m_Params.alpha << "\t" << m_Params.len << "\t" << m_Params.v << "\t" << m_Params.lastlpe << "\t" <<
                         m_Params.fce << "\t" << m_Params.lpe << "\t" << m_Params.fpe << "\t" << m_Params.lse << "\t" << m_Params.fse << "\t" <<
                         m_Params.vce << "\t" << m_Params.vse << "\t" << m_Params.targetFce << "\t" << m_Params.f0 << "\t" << m_Params.err << "\t" <<
                         GetESE() << "\t" << GetEPE() << "\t" << GetPSE() << "\t" << GetPPE() << "\t" << GetPCE() << "\t" <<
                         GetTension() << "\t" << GetLength() << "\t" << GetVelocity() << "\t" <<
                         GetPower() << "\t" << GetMetabolicPower() <<
                         "\n";
    }
}




/* netlib function zeroin original fortran
      double precision function zeroin(ax,bx,f,tol)
      double precision ax,bx,f,tol
c
c      a zero of the function  f(x)  is computed in the interval ax,bx .
c
c  input..
c
c  ax     left endpoint of initial interval
c  bx     right endpoint of initial interval
c  f      function subprogram which evaluates f(x) for any x in
c         the interval  ax,bx
c  tol    desired length of the interval of uncertainty of the
c         final result (.ge.0.)
c
c  output..
c
c  zeroin abscissa approximating a zero of  f  in the interval ax,bx
c
c      it is assumed  that   f(ax)   and   f(bx)   have  opposite  signs
c  this is checked, and an error message is printed if this is not
c  satisfied.   zeroin  returns a zero  x  in the given interval
c  ax,bx  to within a tolerance  4*macheps*abs(x)+tol, where macheps  is
c  the  relative machine precision defined as the smallest representable
c  number such that  1.+macheps .gt. 1.
c      this function subprogram is a slightly  modified  translation  of
c  the algol 60 procedure  zero  given in  richard brent, algorithms for
c  minimization without derivatives, prentice-hall, inc. (1973).
c
      double precision  a,b,c,d,e,eps,fa,fb,fc,tol1,xm,p,q,r,s
      double precision  dabs, d1mach
   10 eps = d1mach(4)
      tol1 = eps+1.0d0
c
      a=ax
      b=bx
      fa=f(a)
      fb=f(b)
c     check that f(ax) and f(bx) have different signs
      if (fa .eq.0.0d0 .or. fb .eq. 0.0d0) go to 20
      if (fa * (fb/dabs(fb)) .le. 0.0d0) go to 20
         write(6,2500)
2500     format(1x,'f(ax) and f(bx) do not have different signs,',
     1             ' zeroin is aborting')
         return
   20 c=a
      fc=fa
      d=b-a
      e=d
   30 if (dabs(fc).ge.dabs(fb)) go to 40
      a=b
      b=c
      c=a
      fa=fb
      fb=fc
      fc=fa
   40 tol1=2.0d0*eps*dabs(b)+0.5d0*tol
      xm = 0.5d0*(c-b)
      if ((dabs(xm).le.tol1).or.(fb.eq.0.0d0)) go to 150
c
c see if a bisection is forced
c
      if ((dabs(e).ge.tol1).and.(dabs(fa).gt.dabs(fb))) go to 50
      d=xm
      e=d
      go to 110
   50 s=fb/fa
      if (a.ne.c) go to 60
c
c linear interpolation
c
      p=2.0d0*xm*s
      q=1.0d0-s
      go to 70
c
c inverse quadratic interpolation
c
   60 q=fa/fc
      r=fb/fc
      p=s*(2.0d0*xm*q*(q-r)-(b-a)*(r-1.0d0))
      q=(q-1.0d0)*(r-1.0d0)*(s-1.0d0)
   70 if (p.le.0.0d0) go to 80
      q=-q
      go to 90
   80 p=-p
   90 s=e
      e=d
      if (((2.0d0*p).ge.(3.0d0*xm*q-dabs(tol1*q))).or.(p.ge.
     *dabs(0.5d0*s*q))) go to 100
      d=p/q
      go to 110
  100 d=xm
      e=d
  110 a=b
      fa=fb
      if (dabs(d).le.tol1) go to 120
      b=b+d
      go to 140
  120 if (xm.le.0.0d0) go to 130
      b=b+tol1
      go to 140
  130 b=b-tol1
  140 fb=f(b)
      if ((fb*(fc/dabs(fc))).gt.0.0d0) go to 20
      go to 30
  150 zeroin=b
      return
      end
*/

/* Standard C source for D1MACH */
/*
#include <stdio.h>
#include <cfloat>
#include <cmath>
double d1mach_(long *i)
{
    switch(*i){
      case 1: return DBL_MIN;
      case 2: return DBL_MAX;
      case 3: return DBL_EPSILON/FLT_RADIX;
      case 4: return DBL_EPSILON;
      case 5: return log10((double)FLT_RADIX);
      }
    fprintf(stderr, "invalid argument: d1mach(%ld)\n", *i);
    exit(1); return 0;
} */

// zeroin.f -- translated by f2c (version 20031025).
// and subsequently edited to improve readability slightly

double zeroin(double *ax, double *bx, double (*f)(double x, void *info), void *info, double *tol)
{
    /* System generated locals */
    double ret_val = 0, d__1, d__2;

    /* Local variables */
    double a, b, c__, d__, e, p, q, r__, s, fa, fb, fc, xm, eps, tol1;

    /*      a zero of the function  f(x)  is computed in the interval ax,bx . */

    /*  input.. */

    /*  ax     left endpoint of initial interval */
    /*  bx     right endpoint of initial interval */
    /*  f      function subprogram which evaluates f(x) for any x in */
    /*         the interval  ax,bx */
    /*  tol    desired length of the interval of uncertainty of the */
    /*         final result (>= 0.) */

    /*  output.. */

    /*  zeroin abscissa approximating a zero of  f  in the interval ax,bx */

    /*      it is assumed  that   f(ax)   and   f(bx)   have  opposite  signs */
    /*  this is checked, and an error message is printed if this is not */
    /*  satisfied.   zeroin  returns a zero  x  in the given interval */
    /*  ax,bx  to within a tolerance  4*macheps*fabs(x)+tol, where macheps  is */
    /*  the  relative machine precision defined as the smallest representable */
    /*  number such that  1.+macheps > 1. */
    /*      this function subprogram is a slightly  modified  translation  of */
    /*  the algol 60 procedure  zero  given in  richard brent, algorithms for */
    /*  minimization without derivatives, prentice-hall, inc. (1973). */

    /* L10: */
    eps = DBL_EPSILON;
    tol1 = eps + 1.;

    a = *ax;
    b = *bx;
    fa = (*f)(a, info);
    fb = (*f)(b, info);
    /*     check that f(ax) and f(bx) have different signs */
    if (fa == 0. || fb == 0.) {
        goto L20;
    }
    if (fa * (fb / fabs(fb)) <= 0.) {
        goto L20;
    }
    fprintf(stderr, "f(ax) and f(bx) do not have different signs, zeroin is aborting\n");
    return ret_val;
L20:
    c__ = a;
    fc = fa;
    d__ = b - a;
    e = d__;
L30:
    if (fabs(fc) >= fabs(fb)) {
        goto L40;
    }
    a = b;
    b = c__;
    c__ = a;
    fa = fb;
    fb = fc;
    fc = fa;
L40:
    tol1 = eps * 2. * fabs(b) + *tol * .5;
    xm = (c__ - b) * .5;
    if (fabs(xm) <= tol1 || fb == 0.) {
        goto L150;
    }

    /* see if a bisection is forced */

    if (fabs(e) >= tol1 && fabs(fa) > fabs(fb)) {
        goto L50;
    }
    d__ = xm;
    e = d__;
    goto L110;
L50:
    s = fb / fa;
    if (a != c__) {
        goto L60;
    }

    /* linear interpolation */

    p = xm * 2. * s;
    q = 1. - s;
    goto L70;

    /* inverse quadratic interpolation */

L60:
    q = fa / fc;
    r__ = fb / fc;
    p = s * (xm * 2. * q * (q - r__) - (b - a) * (r__ - 1.));
    q = (q - 1.) * (r__ - 1.) * (s - 1.);
L70:
    if (p <= 0.) {
        goto L80;
    }
    q = -q;
    goto L90;
L80:
    p = -p;
L90:
    s = e;
    e = d__;
    if (p * 2. >= xm * 3. * q - (d__1 = tol1 * q, fabs(d__1)) || p >= (d__2 =
                                                                       s * .5 * q, fabs(d__2))) {
        goto L100;
    }
    d__ = p / q;
    goto L110;
L100:
    d__ = xm;
    e = d__;
L110:
    a = b;
    fa = fb;
    if (fabs(d__) <= tol1) {
        goto L120;
    }
    b += d__;
    goto L140;
L120:
    if (xm <= 0.) {
        goto L130;
    }
    b += tol1;
    goto L140;
L130:
    b -= tol1;
L140:
    fb = (*f)(b, info);
    if (fb * (fc / fabs(fc)) > 0.) {
        goto L20;
    }
    goto L30;
L150:
    ret_val = b;
    return ret_val;
} /* zeroin_ */



