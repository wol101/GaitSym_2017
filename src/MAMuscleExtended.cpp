/*
 *  MAMuscleExtended.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 21/04/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleExtended - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a parallel spring element

#include "SimpleStrap.h"
#include "MAMuscleExtended.h"
#include "DebugControl.h"
#include "Simulation.h"

// defines for Mathematica CForm output
#define Rule(x,y) x = (y)
#define Sqrt(x) sqrt(x)
#define Power(x, y) pow(x, y)

// constructor

MAMuscleExtended::MAMuscleExtended(Strap *strap): Muscle(strap)
{
    m_Stim = 0;
    m_Act = 0;

    m_ActivationKinetics = true;

    spe = 0; // slack length parallel element (m)
    epe = 0; // elastic constant parallel element (N/m)
    sse = 0; // slack length serial element (m)
    ese = 0; // elastic constant serial element (N/m)

    k = 0; // shape constant
    vmax = 0; // maximum shortening velocity (m/s)
    f0 = 0; // isometric force (N)

    fce = 0; // contractile force
    lpe = 0; // contractile and parallel length
    fpe = 0; // parallel element force
    lse = 0; // serial length
    fse = 0; // serial element force

    vce = 0; // contractile element velocity (m/s)

    lastlpe = -1; // last parallel element length (m) flag value to show that the previous length of the parallel element has not been set

    m_SetActivationFirstTimeFlag = true;

}

// destructor
MAMuscleExtended::~MAMuscleExtended()
{
}

// set the muscle elastic properties
// if tendon length is set to < 0 then set it to a default slack value (done later though)
void MAMuscleExtended::SetSerialElasticProperties(double springConstant, double unloadedLength)
{
    sse = unloadedLength; // slack length serial element (m)
    ese = springConstant; // elastic constant serial element (N/m)
}

void MAMuscleExtended::SetParallelElasticProperties(double springConstant, double unloadedLength)
{
    spe = unloadedLength; // slack length parallel element (m)
    epe = springConstant; // elastic constant parallel element (N/m)
    if (epe < 0) epe = 0; // set flag value to zero
}

// set the muscle contractile properties
void MAMuscleExtended::SetMuscleProperties(double vMax, double F0, double K)
{
    k = K; // shape constant
    vmax = vMax; // maximum shortening velocity (m/s)
    f0 = F0; // isometric force
}

// do any intialisation that relies on the strap being set up properly

void MAMuscleExtended::LateInitialisation()
{
    Muscle::LateInitialisation();

    // lastlpe perhaps not set to anything useful

    double len = m_Strap->GetLength();
    if (epe > 0)
    {
        // handle sse < 0
        if (sse < 0) sse = len - spe;

        vce = 0;
        if (lastlpe < 0) lastlpe = ((m_Act*f0*k*(vce+vmax)+(epe*spe+ese*(len-sse))*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
        else lpe = lastlpe; // use this if user supplied initial fibre length
        /* not needed?
        if (lpe > spe) // pe not slack
        {
            fpe = ((epe*(m_Act*f0*k*(vce+vmax)+ese*(len-spe-sse)*(vce-k*vmax)))/((epe+ese)*(vce-k*vmax)));
            lse = len - lpe;
            fse = ((-(m_Act*ese*f0*k*(vce+vmax))+epe*ese*(len-spe-sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            fce = ((m_Act*f0*k*(vce+vmax))/(-vce+k*vmax));
        }
        else
        {
            if (lastlpe < 0) lpe = (len - sse + (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            lse = len - lpe;
            fse = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
            fpe = (0);
            fce = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
        }
        */
    }
    else
    {
        // handle sse < 0
        if (sse < 0) sse = len - spe;

        vce = 0;
        if (lastlpe < 0) Rule(lpe,-((m_Act*f0)/ese) + len - sse);
        else lpe = lastlpe; // use this if user supplied initial fibre length

        /* not needed?
        lse = len - lpe;
        fce = ese * (lse - sse);
        fse = fce;
        fpe = 0;
        */
    }

    return;
}

// set the proportion of muscle fibres that are active
// calculates the tension in the strap

void MAMuscleExtended::SetActivation(double activation, double timeIncrement)
{
    if (activation < 0) activation = 0;
    else if (activation > 1) activation = 1;
    m_Stim = activation;

    if (m_ActivationKinetics)
    {
        // using activation kinetics from UGM model
        double ft = 0.5; // arbitrary set activation kinetics as 50% fast twitch
        double tact = 80e-3 - 0.47e-3 * ft; // Umberger et al 2003 eq 4
        double tdeact = 90e-3 - 0.56e-3 * ft; // Umberger et al 2003 eq 4
        double t2 = 1 / tdeact;
        double t1 = 1 / tact - t2;
        // Nagano & Gerritsen 2001 A2
        double qdot = (m_Stim - m_Act) * (t1 * m_Stim + t2);
        m_Act += qdot * timeIncrement;
        if (m_Act < 0.001) m_Act = 0.001; // m_Act never drops to zero in practice
    }
    else
        m_Act = m_Stim;

    if (epe > 0) CalculateForceWithParallelElement(timeIncrement);
    else CalculateForceWithoutParallelElement(timeIncrement);
}

void MAMuscleExtended::CalculateForceWithParallelElement(double timeIncrement)
{
    const double goodEnough = 1e-10; // some help for rounding errors

    double alpha = m_Act;
    double len; // total length of system

    len = m_Strap->GetLength();

    int progress = 0;
    while (progress == 0)
    {

        // First assume vce <= 0 (concentric)
        // Solution 1
        Rule(fpe,(epe*(alpha*f0*k + epe*lastlpe + ese*lastlpe + ese*len - epe*spe - 2*ese*spe - ese*sse + epe*k*timeIncrement*vmax +
                       ese*k*timeIncrement*vmax - Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*
                                                       vmax + Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/
             (2.*(epe + ese)));
        Rule(fse,-(ese*(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe + 2*epe*sse + ese*sse +
                        epe*k*timeIncrement*vmax + ese*k*timeIncrement*vmax -
                        Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                             Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/(2.*(epe + ese)));
        Rule(lse,-(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe - ese*sse + epe*k*timeIncrement*vmax +
                   ese*k*timeIncrement*vmax - Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                                                   Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2)))/(2.*(epe + ese)));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 1;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Solution 2
        Rule(fpe,(epe*
                  (alpha*f0*k + epe*lastlpe + ese*lastlpe + ese*len - epe*spe - 2*ese*spe - ese*sse + epe*k*timeIncrement*vmax +
                   ese*k*timeIncrement*vmax + Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*
                                                   vmax + Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/
             (2.*(epe + ese)));
        Rule(fse,-(ese*(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe + 2*epe*sse + ese*sse +
                        epe*k*timeIncrement*vmax + ese*k*timeIncrement*vmax +
                        Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                             Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2))))/(2.*(epe + ese)));
        Rule(lse,-(alpha*f0*k + epe*lastlpe + ese*lastlpe - 2*epe*len - ese*len + epe*spe - ese*sse + epe*k*timeIncrement*vmax +
                   ese*k*timeIncrement*vmax + Sqrt(4*(epe + ese)*k*(alpha*f0 + epe*(lastlpe - spe) + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                                                   Power(alpha*f0*k + epe*(-lastlpe + spe) - ese*(lastlpe - len + sse) + (epe + ese)*k*timeIncrement*vmax,2)))/(2.*(epe + ese)));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 2;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Now try assuming vce > 0 (eccentric)

        // Solution 1
        Rule(fpe,(epe*(-850.5*alpha*f0 - 50.*alpha*f0*k + 472.5*epe*lastlpe + 472.5*ese*lastlpe + 472.5*ese*len - 472.5*epe*spe -
                       945.*ese*spe - 472.5*ese*sse - 62.5*epe*k*timeIncrement*vmax - 62.5*ese*k*timeIncrement*vmax -
                       0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                               Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                               Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                             1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                               epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                        lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                               236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                          epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                               15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                               alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                              k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                         ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                              Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(fse,(ese*(850.5*alpha*f0 + 50.*alpha*f0*k + epe*(-472.5*lastlpe + 945.*len - 472.5*spe - 945.*sse) +
                       ese*(-472.5*lastlpe + 472.5*len - 472.5*sse) + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax +
                       0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                               Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                               Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                             1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                               epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                        lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                               236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                          epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                               15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                               alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                              k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                                         ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax))
                                                         ))/Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(lse,(850.5*alpha*f0 + 50.*alpha*f0*k - 472.5*epe*lastlpe - 472.5*ese*lastlpe + 945.*epe*len + 472.5*ese*len - 472.5*epe*spe +
                  472.5*ese*sse + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax +
                  0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                          Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                          Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                        1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                          epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                   lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                          236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                     epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                          15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                          alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                         k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                    ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                         k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                         Power(timeIncrement,2)))/(945.*epe + 945.*ese));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 3;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // Solution 2
        Rule(fpe,
             (epe*(-850.5*alpha*f0 - 50.*alpha*f0*k + 472.5*epe*lastlpe + 472.5*ese*lastlpe + 472.5*ese*len - 472.5*epe*spe - 945.*ese*spe -
                   472.5*ese*sse - 62.5*epe*k*timeIncrement*vmax - 62.5*ese*k*timeIncrement*vmax +
                   0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                           Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                           Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                         1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                           epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                    lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                           236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                      epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                           15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                           alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                          k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                     ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                          k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                          Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(fse,(ese*(850.5*alpha*f0 + 50.*alpha*f0*k + epe*(-472.5*lastlpe + 945.*len - 472.5*spe - 945.*sse) +
                       ese*(-472.5*lastlpe + 472.5*len - 472.5*sse) + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax -
                       0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                               Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                               Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                             1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                               epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                        lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                               236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                          epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                               15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                               alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                              k*(-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax) +
                                                         ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                              k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 24999.999999999993*k)*timeIncrement*vmax))
                                                         ))/Power(timeIncrement,2))))/(945.*epe + 945.*ese));
        Rule(lse,(850.5*alpha*f0 + 50.*alpha*f0*k - 472.5*epe*lastlpe - 472.5*ese*lastlpe + 945.*epe*len + 472.5*ese*len - 472.5*epe*spe +
                  472.5*ese*sse + 62.5*epe*k*timeIncrement*vmax + 62.5*ese*k*timeIncrement*vmax -
                  0.5*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                          Power(epe,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*spe + 893025.0000000001*Power(spe,2)) +
                                          Power(ese,2)*(893025.0000000001*Power(lastlpe,2) - 1.7860500000000002e6*lastlpe*len + 893025.0000000001*Power(len,2) +
                                                        1.7860500000000002e6*lastlpe*sse - 1.7860500000000002e6*len*sse + 893025.0000000001*Power(sse,2)) +
                                          epe*ese*(1.7860500000000002e6*Power(lastlpe,2) + 1.7860500000000002e6*len*spe - 1.7860500000000002e6*spe*sse +
                                                   lastlpe*(-1.7860500000000002e6*len - 1.7860500000000002e6*spe + 1.7860500000000002e6*sse)) -
                                          236250.*k*(Power(epe,2)*(1.*lastlpe - 1.*spe) + Power(ese,2)*(1.*lastlpe - 1.*len + 1.*sse) +
                                                     epe*ese*(2.*lastlpe - 1.*len - 1.*spe + 1.*sse))*timeIncrement*vmax +
                                          15625.*(1.*Power(epe,2) + 2.*epe*ese + 1.*Power(ese,2))*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                          alpha*f0*(epe*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe + (-3.214890000000001e6 - 189000.00000000003*k)*spe +
                                                         k*(-47250. + 24999.999999999993*k)*timeIncrement*vmax) +
                                                    ese*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                                         k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 24999.999999999993*k)*timeIncrement*vmax))))/
                                         Power(timeIncrement,2)))/(945.*epe + 945.*ese));

        lpe = len - lse;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 4;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= spe - goodEnough) break; // check consistency

        // now consider special case if (lpe < spe) // pe slack special case

        // First assume vce <= 0 (concentric)
        // Solution 1
        Rule(lpe,(alpha*f0*k + ese*(lastlpe + len - sse + k*timeIncrement*vmax) -
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*ese));
        Rule(fpe,0);
        Rule(fse,(-(alpha*f0*k) - ese*(lastlpe - len + sse + k*timeIncrement*vmax) +
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/2.);

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 5;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Solution 2
        Rule(lpe,(alpha*f0*k + ese*(lastlpe + len - sse + k*timeIncrement*vmax) +
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*ese));
        Rule(fpe,0);
        Rule(fse,(-(alpha*f0*k) - ese*(lastlpe - len + sse + k*timeIncrement*vmax) -
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/2.);

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 6;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Now check if (vce > 0) (eccentric)

        // Solution 1
        Rule(lpe,(alpha*f0*(-0.9 - 0.052910052910052914*k) + 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse -
                  0.06613756613756615*ese*k*timeIncrement*vmax - 0.0005291005291005291*timeIncrement*
                  Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                        Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                      893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax -
                                      236250.00000000006*k*sse*timeIncrement*vmax + 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                      lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                        alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                       Power(timeIncrement,2)))/ese);
        Rule(fpe,0.);
        Rule(fse,
             alpha*f0*(0.9 + 0.052910052910052914*k) - 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse +
             0.06613756613756615*ese*k*timeIncrement*vmax + 0.0005291005291005291*timeIncrement*
             Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                   Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                 893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax - 236250.00000000006*k*sse*timeIncrement*vmax +
                                 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                 lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                   alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                 k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                  Power(timeIncrement,2)));

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 7;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // Solution 2
        Rule(lpe,(alpha*f0*(-0.9 - 0.052910052910052914*k) + 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse -
                  0.06613756613756615*ese*k*timeIncrement*vmax + 0.0005291005291005291*timeIncrement*
                  Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                        Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                      893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax -
                                      236250.00000000006*k*sse*timeIncrement*vmax + 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                      lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                        alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                      k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                       Power(timeIncrement,2)))/ese);
        Rule(fpe,0.);
        Rule(fse,
             alpha*f0*(0.9 + 0.052910052910052914*k) - 0.5*ese*lastlpe + 0.5*ese*len - 0.5*ese*sse +
             0.06613756613756615*ese*k*timeIncrement*vmax - 0.0005291005291005291*timeIncrement*
             Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                   Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000005*Power(len,2) - 1.786050000000001e6*len*sse +
                                 893025.0000000005*Power(sse,2) + 236250.00000000006*k*len*timeIncrement*vmax - 236250.00000000006*k*sse*timeIncrement*vmax +
                                 15624.999999999998*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                 lastlpe*(-1.786050000000001e6*len + 1.786050000000001e6*sse - 236250.00000000006*k*timeIncrement*vmax)) +
                   alpha*ese*f0*((3.214890000000001e6 + 189000.00000000003*k)*lastlpe - 3.214890000000001e6*len + 3.214890000000001e6*sse +
                                 k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250.00000000001 + 25000.000000000004*k)*timeIncrement*vmax)))/
                  Power(timeIncrement,2)));

        lse = len - lpe;
        fce = fse - fpe;
        vce = (lpe - lastlpe) / timeIncrement;

        progress = 8;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe <= spe + goodEnough) break; // check consistency

        // no consistent result found - usually because vce is out of range
        // we shouldn't get here too often but a few time - especially at the beginning of a simulation
        // or after an impact should be OK
        progress = 9;
        if (m_Strap->GetVelocity() > vmax) vce = vmax;
        else if (m_Strap->GetVelocity() < -vmax) vce = -vmax;
        else vce = 0;
        lpe = ((m_Act*f0*k*(vce+vmax)+(epe*spe+ese*(len-sse))*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
        if (lpe > spe) // pe not slack
        {
            fpe = ((epe*(m_Act*f0*k*(vce+vmax)+ese*(len-spe-sse)*(vce-k*vmax)))/((epe+ese)*(vce-k*vmax)));
            fse = ((-(m_Act*ese*f0*k*(vce+vmax))+epe*ese*(len-spe-sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            /*lse = ((-(m_Act*f0*k*(vce+vmax))+(epe*(len-spe)+ese*sse)*(vce-k*vmax))/((epe+ese)*(vce-k*vmax)));
            fce = ((m_Act*f0*k*(vce+vmax))/(-vce+k*vmax));*/
        }
        else
        {
            progress = 10;
            lpe = (len - sse + (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            fse = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));
            fpe = (0);
            /*lse = (sse - (m_Act*f0*k*(vce + vmax))/(ese*(vce - k*vmax)));
            fce = ((m_Act*f0*k*(vce + vmax))/(-vce + k*vmax));*/
        }
        lse = len - lpe;
        fce = fse - fpe;
        if (fce < 0) fce = 0; // sanity check
        if (fse < 0) fse = 0; // sanity check
    }

    lastlpe = lpe;
    m_Strap->SetTension(fse);

    if (gDebug == MAMuscleExtendedDebug)
    {
        if (DebugFilters("SetActivation", m_Name))
        {
            static int m_SetActivationFirstTimeFlag = true;
            if (m_SetActivationFirstTimeFlag) // only happens once
            {
                m_SetActivationFirstTimeFlag = false;
                *gDebugStream << "MAMuscleExtended::SetActivation " <<
                        " progress " <<
                        " m_Stim " <<
                        " m_Act " <<
                        " len " <<
                        " fpe " <<
                        " lpe " <<
                        " lse " <<
                        " fse " <<
                        " fce " <<
                        " m_Velocity " <<
                        " m_Tension " <<
                        " vce " <<
                        " serialStrainEnergy " <<
                        " parallelStrainEnergy " <<
                        "\n";
            }

            double serialStrainEnergy = 0.5 * (lse - sse) * (lse - sse) * ese;
            double parallelStrainEnergy = 0.5 * (lpe - spe) * (lpe - spe) * epe;

            *gDebugStream << m_Name <<
                    " " << progress <<
                    " " << m_Stim <<
                    " " << m_Act <<
                    " " << len <<
                    " " << fpe <<
                    " " << lpe <<
                    " " << lse <<
                    " " << fse <<
                    " " << fce <<
                    " " << m_Strap->GetVelocity() <<
                    " " << m_Strap->GetTension() <<
                    " " << vce <<
                    " " << serialStrainEnergy <<
                    " " << parallelStrainEnergy <<
                    "\n";
        }
    }
}


void MAMuscleExtended::CalculateForceWithoutParallelElement(double timeIncrement)
{
    const double goodEnough = 1e-10; // some help for rounding errors

    double alpha = m_Act;
    double len; // total length of system

    len = m_Strap->GetLength();

    int progress = 0;
    while (progress == 0)
    {

        // First assume vce <= 0 (concentric)
        // Solution 1
        Rule(lse,(-(alpha*f0*k) - ese*lastlpe + ese*len + ese*sse - ese*k*timeIncrement*vmax +
                  Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                       Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*ese));
        fce = ese * (lse - sse);
        fse = fce;
        lpe = len - lse;
        fpe = 0;
        vce = (lpe - lastlpe) / timeIncrement;
        progress = 1;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= 0 - goodEnough) break; // check consistency

        // Solution 2
        Rule(lse,-(alpha*f0*k + ese*(lastlpe - len - sse + k*timeIncrement*vmax) +
                   Sqrt(4*ese*k*(alpha*f0 + ese*(lastlpe - len + sse))*timeIncrement*vmax +
                        Power(alpha*f0*k + ese*(-lastlpe + len - sse + k*timeIncrement*vmax),2)))/(2.*ese));
        fce = ese * (lse - sse);
        fse = fce;
        lpe = len - lse;
        fpe = 0;
        vce = (lpe - lastlpe) / timeIncrement;
        progress = 2;
        if (vce <= 0 + goodEnough && fce >= 0 - goodEnough && lpe >= 0 - goodEnough) break; // check consistency

        // Now try assuming vce > 0 (eccentric)

        // Solution 1
        Rule(lse,(alpha*f0*(0.9 + 0.052910052910052914*k) - 0.5*ese*lastlpe + 0.5*ese*len + 0.5*ese*sse + 0.06613756613756615*ese*k*timeIncrement*vmax +
                  0.0005291005291005291*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                                            Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000001*Power(len,2) - 1.7860500000000002e6*len*sse +
                                                                          893025.0000000001*Power(sse,2) + 236250.00000000003*k*len*timeIncrement*vmax - 236250.00000000003*k*sse*timeIncrement*vmax +
                                                                          15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                                          lastlpe*(-1.7860500000000002e6*len + 1.7860500000000002e6*sse - 236250.00000000003*k*timeIncrement*vmax)) +
                                                            alpha*ese*f0*((3.2148900000000005e6 + 189000.00000000003*k)*lastlpe - 3.2148900000000005e6*len + 3.2148900000000005e6*sse +
                                                                          k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 25000.*k)*timeIncrement*vmax)))/Power(timeIncrement,2)))/ese);
        fce = ese * (lse - sse);
        fse = fce;
        lpe = len - lse;
        fpe = 0;
        vce = (lpe - lastlpe) / timeIncrement;
        progress = 3;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= 0) break; // check consistency

        // Solution 2
        Rule(lse,(alpha*f0*(0.9 + 0.052910052910052914*k) - 0.5*ese*lastlpe + 0.5*ese*len + 0.5*ese*sse + 0.06613756613756615*ese*k*timeIncrement*vmax -
                  0.0005291005291005291*timeIncrement*Sqrt((10000.*Power(alpha,2)*Power(f0,2)*(17.009999590960987 + k)*(17.010000409039023 + k) +
                                                            Power(ese,2)*(893025.0000000001*Power(lastlpe,2) + 893025.0000000001*Power(len,2) - 1.7860500000000002e6*len*sse +
                                                                          893025.0000000001*Power(sse,2) + 236250.00000000003*k*len*timeIncrement*vmax - 236250.00000000003*k*sse*timeIncrement*vmax +
                                                                          15625.*Power(k,2)*Power(timeIncrement,2)*Power(vmax,2) +
                                                                          lastlpe*(-1.7860500000000002e6*len + 1.7860500000000002e6*sse - 236250.00000000003*k*timeIncrement*vmax)) +
                                                            alpha*ese*f0*((3.2148900000000005e6 + 189000.00000000003*k)*lastlpe - 3.2148900000000005e6*len + 3.2148900000000005e6*sse +
                                                                          k*(-189000.00000000003*len + 189000.00000000003*sse + (-47250. + 25000.*k)*timeIncrement*vmax)))/Power(timeIncrement,2)))/ese);
        fce = ese * (lse - sse);
        fse = fce;
        lpe = len - lse;
        fpe = 0;
        vce = (lpe - lastlpe) / timeIncrement;
        progress = 4;
        if (vce >= 0 - goodEnough && fce >= 0 - goodEnough && lpe >= 0) break; // check consistency

         // no consistent result found - usually because vce is out of range
        // we shouldn't get here too often but a few time - especially at the beginning of a simulation
        // or after an impact should be OK
        progress = 5;
        if (m_Strap->GetVelocity() > vmax) vce = vmax;
        else if (m_Strap->GetVelocity() < -vmax) vce = -vmax;
        else vce = 0;

        if (vce <= 0) Rule(lse,sse + (alpha*f0*k*(vce + vmax))/(ese*(-vce + k*vmax)));
        else Rule(lse,1.*sse + (alpha*f0*((13.607999999999999 + 0.8*k)*vce + 1.*k*vmax))/(ese*(7.56*vce + k*vmax)));
        fce = ese * (lse - sse);
        fse = fce;
        lpe = len - lse;
        fpe = 0;
        vce = (lpe - lastlpe) / timeIncrement;


        if (fce < 0) fce = 0; // sanity check
        if (fse < 0) fse = 0; // sanity check
    }

    lastlpe = lpe;
    m_Strap->SetTension(fse);

    if (gDebug == MAMuscleExtendedDebug)
    {
        if (DebugFilters("SetActivation", m_Name))
        {
            if (m_SetActivationFirstTimeFlag) // only happens once
            {
                m_SetActivationFirstTimeFlag = false;
                *gDebugStream << "MAMuscleExtended::SetActivation " <<
                        " progress " <<
                        " m_Stim " <<
                        " m_Act " <<
                        " len " <<
                        " fpe " <<
                        " lpe " <<
                        " lse " <<
                        " fse " <<
                        " fce " <<
                        " m_Velocity " <<
                        " m_Tension " <<
                        " vce " <<
                        " serialStrainEnergy " <<
                        " parallelStrainEnergy " <<
                        "\n";
            }

            double serialStrainEnergy = 0.5 * (lse - sse) * (lse - sse) * ese;
            double parallelStrainEnergy = 0.5 * (lpe - spe) * (lpe - spe) * epe;

            *gDebugStream << m_Name <<
                    " " << progress <<
                    " " << m_Stim <<
                    " " << m_Act <<
                    " " << len <<
                    " " << fpe <<
                    " " << lpe <<
                    " " << lse <<
                    " " << fse <<
                    " " << fce <<
                    " " << m_Strap->GetVelocity() <<
                    " " << m_Strap->GetTension() <<
                    " " << vce <<
                    " " << serialStrainEnergy <<
                    " " << parallelStrainEnergy <<
                    "\n";
        }
    }
}


// calculate the metabolic power of the muscle

double MAMuscleExtended::GetMetabolicPower()
{
    // m_Velocity is negative when muscle shortening
    // we need the sign the other way round
    double relV = -vce / vmax;

    // limit relV
    if (relV > 1) relV = 1;
    else if (relV < -1) relV = -1;

    double relVSquared = relV * relV;
    double relVCubed = relVSquared * relV;

    double sigma = (0.054 + 0.506 * relV + 2.46 * relVSquared) /
                   (1 - 1.13 * relV + 12.8 * relVSquared - 1.64 * relVCubed);

    if (gDebug == MAMuscleExtendedDebug)
    {
        if (DebugFilters("GetMetabolicPower", m_Name))
        {
            *gDebugStream << "MAMuscle::GetMetabolicPower " << m_Name <<
                    " m_Act " << m_Act <<
                    " f0 " << f0 <<
                    " vmax " << vmax <<
                    " m_Velocity " << m_Strap->GetVelocity() <<
                    " sigma " << sigma <<
                    " power " << m_Act * f0 * vmax * sigma << "\n";
        }
    }
    return (m_Act * f0 * vmax * sigma);
}

void MAMuscleExtended::Dump()
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
            *m_DumpStream << "Time\tstim\tact\tspe\tepe\tsse\tese\tk\tvmax\tf0\tfce\tlpe\tfpe\tlse\tfse\tvce\tvse\tense\tenpe\tpse\tppe\tpce\ttension\tlength\tvelocity\tPMECH\tPMET\n";
        }
    }


    if (m_DumpStream)
    {
        *m_DumpStream << m_simulation->GetTime() << "\t" <<m_Stim << "\t" << m_Act << "\t" << spe << "\t" <<
                epe << "\t" << sse << "\t" << ese << "\t" << k << "\t" << vmax << "\t" << f0 << "\t" <<
                fce << "\t" << lpe << "\t" << fpe << "\t" << lse << "\t" << fse << "\t" << vce << "\t" <<
                GetVSE() << "\t" << GetESE() << "\t" << GetEPE() << "\t" << GetPSE() << "\t" << GetPPE() << "\t" << GetPCE() <<
                "\t" << m_Strap->GetTension() << "\t" << m_Strap->GetLength() << "\t" << m_Strap->GetVelocity() <<
                "\t" << m_Strap->GetVelocity() * m_Strap->GetTension() << "\t" << GetMetabolicPower() <<
                "\n";
    }
}

