/*
 *  UGMMuscle.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Mon Aug 16 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 *  This muscle model is based on:
 *  Umberger BR, Gerritsen KG, Martin PE.
 *  A model of human muscle energy expenditure.
 *  Comput Methods Biomech Biomed Engin. 2003 Apr;6(2):99-111.
 */

#include <iostream>
#include <string>

#include "Strap.h"
#include "Util.h"
#include "DebugControl.h"
#include "UGMMuscle.h"
#include "Simulation.h"

// constructor
UGMMuscle::UGMMuscle(Strap *strap): Muscle(strap)
{
    // modelling constants have the following default values
    // Umberger et al 2003
    m_specifictension = 0.25e6;
    m_vmaxft = 12;
    m_density = 1059.7;

    // these values are constants from Nagano & Gerritsen 2001
    m_fmaxecc = 1.5; // Fasympt
    m_slopfac = 2.0; // Slopefactor

    // activation levels
    m_act = 0;
    m_stim = 0;

    // aerobic by default
    m_s = 1.5;

    // and flag that we have just been created
    m_newObject = true;
}

// destructor
UGMMuscle::~UGMMuscle()
{
}

// set the proportion of fast twitch fibres and all subsequent timing values
// also sets the heat rate coefficients
void UGMMuscle::SetFibreComposition(double fastTwitchFraction)
{
    m_ft = fastTwitchFraction;
    m_arel = 0.1 + 0.4 * m_ft; // Umberger et al 2003 eq 1
    m_brel = m_arel * m_vmaxft; // Umberger et al 2003 eq 2
    m_tact = 80e-3 - 0.47e-3 * m_ft; // Umberger et al 2003 eq 4
    m_tdeact = 90e-3 - 0.56e-3 * m_ft; // Umberger et al 2003 eq 4

    //c--- Caculate activation/maintenence heat rate
    //c    based on %FT fibers (after bolstad & ersland)
    m_amh = 1.28 * m_ft + 25.0;

    //c--- shortening heat rate coefficient
    //c    4.0 x m_amh of st fibers at vmax of st fibers
    //c    0.5 x m_amh of m_FT fibers at vmax of m_FT fibers (barclay et al)
    m_vmaxst = m_vmaxft * 0.407090531; //0.05^0.3;
    m_shst   = 4.0 *  25.0 / m_vmaxst;
    m_shft   = 1.0 * 153.0 / m_vmaxft;

    //c--- lengthening heat rate coefficient
    //c    4.0 x sh (approximates constable et al. and hawkins & mole)
    m_lh = 4.0 * m_shst;
}

// set the muscle geometry
// Nagano & Gerritsen 2001
// PCSA - physiological cross section area (m2)
// optimalLength - usually resting fibre length (m)
// relativeWidth - maximum length range of force production relative to optimalLength (note this value is effectively the half width - typical value 0.5)
void UGMMuscle::SetMuscleGeometry(double pcsa, double optimumFibreLength, double relativeWidth, double tendonLength,
                                  StrainModel serialStrainModel, double serialStrainAtFmax,
                                  StrainModel parallelStrainModel, double parallelStrainAtFmax)
{
    m_pcsa = pcsa;
    m_lceopt = optimumFibreLength;
    m_lce = m_lceopt;
    m_vce = 0;
    m_fmax = m_specifictension * m_pcsa;
    m_width = relativeWidth;
    m_mass = m_pcsa * m_lceopt * m_density;
    m_tendonlength = tendonLength;

    m_parallelStrainModel = parallelStrainModel;
    m_parallelStrainAtFmax = parallelStrainAtFmax;
    switch (m_parallelStrainModel)
    {
        case linear:
            m_kpe = m_fmax/(m_lceopt*m_parallelStrainAtFmax);
            break;
        case square:
            m_kpe = m_fmax/(SQUARE(m_lceopt*m_parallelStrainAtFmax));
    }
    m_serialStrainModel = serialStrainModel;
    m_serialStrainAtFmax = serialStrainAtFmax;
    switch (m_serialStrainModel)
    {
        case linear:
            m_kse = m_fmax/(m_tendonlength*m_serialStrainAtFmax);
            break;
        case square:
            m_kse = m_fmax/(SQUARE(m_tendonlength*m_serialStrainAtFmax));
    }
}

// return the serial elastic energy storage
double UGMMuscle::GetESE()
{
    double energy;
    double delLen = m_Strap->GetLength() - m_lce - m_tendonlength;
    if (delLen < 0) return 0;

    switch (m_serialStrainModel)
    {
        case linear:
            energy = 0.5 * SQUARE(delLen) * m_kse;
            break;
        case square:
            energy = 0.333333333333333 * pow(delLen, 3) * m_kse;
    }
    return energy;
}

// return the parallel elastic energy storage
double UGMMuscle::GetEPE()
{
    double energy;
    double delLen = m_lce - m_lceopt;
    if (delLen < 0) return 0;

    switch (m_parallelStrainModel)
    {
        case linear:
            energy = 0.5 * SQUARE(delLen) * m_kpe;
            break;
        case square:
            energy = 0.333333333333333 * pow(delLen, 3) * m_kpe;
    }
    return energy;
}

// set some modelling constants
// these need to be set before SetFibreComposition and SetMuscleGeometry
void UGMMuscle::SetModellingConstants(double specificTension, double relativeContractionVelocity, double muscleDensity)
{
    m_specifictension = specificTension;
    m_vmaxft = relativeContractionVelocity;
    m_density = muscleDensity;
}

// set the activation level
// this code directly modified from the fortran
// but missing the pennation angle sections
void UGMMuscle::SetStim(double stim, double timeIncrement)
{
    // set the internal activation level
    m_stim = MAX(stim, 0.00001); // modified from Fortran
                                 // Umberger et al 2003 eq 3
    double t2 = 1 / m_tdeact;
    double t1 = 1 / m_tact - t2;
    // Nagano & Gerritsen 2001 A2
    double qdot = (m_stim - m_act) * (t1 * m_stim + t2);
    m_act += qdot * timeIncrement;
    // minimum value for m_ACT now needed
    m_act = MAX(m_act, 0.0001);

    // late initialisation
    double peext = 0, seext = 0, fse;
    if (m_newObject)
    {
        m_newObject = false;
        double ext = m_Strap->GetLength() - (m_tendonlength + m_lceopt);
        if (ext <= 0)
            m_lce = m_Strap->GetLength() - m_tendonlength;
        else // need to partition the stretch between the two elastic components
        {
            if (m_serialStrainModel == square && m_parallelStrainModel == square)
            {
                seext = (ext*sqrt(m_kpe))/(sqrt(m_kpe) - sqrt(m_kse));
                peext = ext - seext;
                if (seext <= 0 || peext <=  0)
                {
                    seext = (ext*sqrt(m_kpe))/(sqrt(m_kpe) + sqrt(m_kse));
                    peext = ext - seext;
                }
            }
            else if (m_serialStrainModel == linear && m_parallelStrainModel == linear)
            {
                seext = (ext*m_kpe)/(m_kpe + m_kse);
                peext = ext - seext;
            }
            else if (m_serialStrainModel == square && m_parallelStrainModel == linear)
            {
                seext = (-m_kpe + sqrt(m_kpe)*sqrt(m_kpe + 4*ext*m_kse))/(2.*m_kse);
                peext = ext - seext;
                if (seext <= 0 || peext <= 0)
                {
                    seext = -(m_kpe + sqrt(m_kpe)*sqrt(m_kpe + 4*ext*m_kse))/(2.*m_kse);
                    peext = ext - seext;
                }
            }
            else if (m_serialStrainModel == linear && m_parallelStrainModel == square)
            {
                seext = (2*ext*m_kpe + m_kse - sqrt(m_kse)*sqrt(4*ext*m_kpe + m_kse))/(2.*m_kpe);
                peext = ext - seext;
                if (seext <= 0 || peext <= 0)
                {
                    seext = (2*ext*m_kpe + m_kse + sqrt(m_kse)*sqrt(4*ext*m_kpe + m_kse))/(2.*m_kpe);
                    peext = ext - seext;
                }
            }
            else std::cerr << "Unrecognized combination of strain models\n";
            if (seext <= 0 || peext <= 0) std::cerr << "Error calculating extensions\n";
            m_lce = m_lceopt + peext;
        }
    }

    // Note removing pennation angle effects
    // c--- Series elastic element force
    seext = MAX(m_Strap->GetLength() - m_lce - m_tendonlength, 0);
    switch (m_serialStrainModel)
    {
        case linear:
            fse = m_kse * seext;
            break;
        case square:
            fse = m_kse * SQUARE(seext);
    }
    m_Strap->SetTension(fse); // tension in unit is same as tension in serial elastic element
                     // c--- Parallel elastic element force
    peext = MAX(m_lce - m_lceopt, 0);
    double fpe;
    switch (m_parallelStrainModel)
    {
        case linear:
            fpe = m_kpe * peext;
            break;
        case square:
            fpe = m_kpe * SQUARE(peext);
    }
    // c--- Contractile element force
    m_fce = fse - fpe;

    // calculate isometric force
    // Nagano & Gerritsen 2001 A5 & A6
    // c--- Normalized contractile element force-length curve
    //double lce = m_Length - m_tendonLength;
    double c0 = -1 / SQUARE(m_width);
    double relLen = m_lce / m_lceopt;
    m_fiso = c0 * SQUARE(relLen) - 2 * c0 * relLen + c0 + 1;
    m_fiso = MAX(m_fiso, 1e-5);

    // Contracile element velocity
    // this is the modified version after email 3
    double Afact = pow(m_act, -0.3);
    double arel = Afact * m_arel;
    if (m_lce >= m_lceopt)
        arel = arel * m_fiso;

    bool concentricFlag = false;
    double num, den;
    // c    concentric
    if (m_fce <= (m_fiso*m_fmax*m_act))
    {
        concentricFlag = true;
        /* this is the original code but it is sensitive to out of range values
        m_vce = -1*m_lceopt*((((m_fiso+arel)*m_brel)/(m_fce/(m_fmax*m_act)+arel))-m_brel);
        */
        /* However this is just a rearrangement that is easier to check (see mathematica file) */
        num = m_brel*(m_fce - m_act*m_fiso*m_fmax)*m_lceopt;
        den = m_fce + arel*m_act*m_fmax;
        if (num >= 0)
        {
            m_vce = 0;
            std::cerr << "Should never get here\n";
            std::cerr << "Applying concentric m_vce = 0 fixup to " << m_Name << "\n";
        }
        else if (den <= 0)
        {
            m_vce = -m_vmaxft * m_lceopt;
            if (gDebug == UGMMuscleDebug) *gDebugStream << "Applying concentric m_vce = -m_vmaxft * m_lceopt fixup to " << m_Name << "\n";
        }
        else
        {
            m_vce = num / den;
        }
    }
    // c    eccentric
    else
    {
        /* this is the original code but it is sensitive to out of range values
        double c1, c2, c3;
        c2 = -1*m_fiso*m_fmaxecc;
        c1 = (m_brel*SQUARE(m_fiso)*SQUARE(1-m_fmaxecc))/(m_slopfac*(m_fiso+arel));
        c3 = -1*((m_brel*m_fiso*(1-m_fmaxecc))/(m_slopfac*(m_fiso+arel)));
        // c    hyperbolic formula
        if ((m_fce/(m_fmax*m_act)) <= (-1*sqrt(c1/sloplin)-c2))
    {
            m_vce = -1*m_lceopt*(c1/(m_fce/(m_fmax*m_act)+c2)+c3);
    }
        else
    {
            // c    linear asymptote
            m_vce = m_lceopt*sloplin*(m_fce/(m_fmax*m_act)+sqrt(c1/sloplin)+c2)+m_lceopt*(sqrt(c1*sloplin)-c3);
    }
        */
        /* However this is just a rearrangement that is easier to check (see mathematica file) */
        num = m_brel*m_fiso*(-m_fce + m_act*m_fiso*m_fmax)*(-1 + m_fmaxecc)*m_lceopt;
        den = (arel + m_fiso)*(m_fce - m_act*m_fiso*m_fmax*m_fmaxecc)*m_slopfac;
        if (num >= 0)
        {
            m_vce = 0;
            std::cerr << "Should never get here\n";
            std::cerr << "Applying eccentric m_vce = 0 fixup to " << m_Name << "\n";;
        }
        else if (den >= 0)
        {
            m_vce = m_vmaxft * m_lceopt;
            if (gDebug == UGMMuscleDebug) *gDebugStream << "Applying eccentric m_vce = m_vmaxft * m_lceopt fixup to " << m_Name << "\n";
        }
        else
        {
            m_vce = num / den;
        }
    }

    // adjust muscle length
    double speedLimit = m_vmaxft * m_lceopt;
    if (m_vce < -speedLimit)
    {
        if (gDebug == UGMMuscleDebug) *gDebugStream << "Negative speed limit fix applied\n";
        m_vce = -speedLimit;
    }
    else if (m_vce > speedLimit)
    {
        if (gDebug == UGMMuscleDebug) *gDebugStream << "Positive speed limit fix applied\n";
        m_vce = speedLimit;
    }
    m_lce += m_vce * timeIncrement;
    if (m_lce < 0)
    {
        if (gDebug == UGMMuscleDebug) *gDebugStream << "m_vce < 0 fix applied\n";
        m_lce = 0;
    }

    if (gDebug == UGMMuscleDebug)
    {
        *gDebugStream << "MAMuscle::SetStim " << m_Name << " "
        << "m_act " << m_act << " "
        << "m_stim " << m_stim << " "
        << "m_fiso " << m_fiso << " "
        << "m_lce " << m_lce << " "
        << "m_vce " << m_vce << " "
        << "concentricFlag " << concentricFlag << " "
        << "m_fce " << m_fce << " "
        << "fse " << fse << " "
        << "fpe " << fpe << " "
        << "m_Tension " << m_Strap->GetTension() << " "
        << "m_Length " << m_Strap->GetLength() << " "
        << "m_Velocity " << m_Strap->GetVelocity() << "\n";
    }
}

double UGMMuscle::GetMetabolicPower()
{
    // this function converted directly from fortran
    // altering variable names as required

    double lcerel = m_lce/m_lceopt;
    double vcerel = m_vce/m_lceopt;

    //c--- Heat rate rises quickly upon excitation
    //c    and falls slowly upon deactivation
    double ea;
    if (m_stim > m_act)
        ea = m_stim;
    else
        ea = (m_stim + m_act) * 0.5;

    //c--- Nonlinear scaling of amhdot & slhdot
    double eamh = pow(ea, 0.6);
    double eash = pow(ea, 2.0);

    // What follows is eq 18, although the nesting of the if's has
    // been reversed relative to the manuscript.
    double amhdot, slhdtst, slhdtft, slhdot, cehdot, cewdot;
    //c--- CE shortening
    if (vcerel <= 0.0)
    {
        if (lcerel > 1.0)
        {
            amhdot = (0.4*m_amh + 0.6*m_amh*m_fiso)*eamh;
            slhdtst = -1.0*m_shst*vcerel*m_fiso*eash;
            if (slhdtst > m_shst*m_vmaxst*eash)
                slhdtst = m_shst*m_vmaxst*eash;
            slhdtst = slhdtst*(1.0-m_ft/100.0);
            slhdtft = -1.0*(m_shft*vcerel*m_fiso*eash)*(m_ft/100.0);
            slhdot = slhdtst + slhdtft;
        }
        else // if (lcerel <= 1.0)
        {
            amhdot = m_amh*eamh;
            slhdtst = -1.0*m_shst*vcerel*eash;
            if (slhdtst > m_shst*m_vmaxst*eash)
                slhdtst = m_shst*m_vmaxst*eash;
            slhdtst = slhdtst*(1.0-m_ft/100.0);
            slhdtft = -1.0*(m_shft*vcerel*eash)*(m_ft/100.0);
            slhdot = slhdtst + slhdtft;
        }
    }
    //c--- CE lengthening
    else // if (vcerel > 0.0)
    {
        if (lcerel > 1.0)
        {
            amhdot = (0.4*m_amh + 0.6*m_amh*m_fiso)*eamh;
            slhdot = (m_lh*vcerel)*m_fiso*ea;
        }
        else // if (lcerel <= 1.0)
        {
            amhdot = m_amh*eamh;
            slhdot = (m_lh*vcerel)*ea;
        }
    }
    //c--- Account for muscle basal metabolic rate
    amhdot = MAX(1.0,amhdot);
    //c--- Sum activation, maintenece, & shortening heat rates,
    //c    and apply aerobic/anaerobic scaling factor
    cehdot = (amhdot + slhdot)*m_s;
    //c--- CE mechanical power output, normalized to muscle mass
    cewdot = -1.0 * (m_vce * m_fce / m_mass);

    double metabolicPower;
    if (m_allowReverseWork)
    {
        metabolicPower = (cehdot + cewdot) * m_mass;
    }
    else
    {
        // from Constable et al 1997 it seems that the 'lost' energy in eccentric
        // contraction is not converted into ATP for reuse so do not add negative
        // values of cewdot
        if (cewdot > 0) metabolicPower = (cehdot + cewdot) * m_mass;
        else metabolicPower = cehdot * m_mass;
    }

    if (gDebug == UGMMuscleDebug)
    {
        *gDebugStream << "MAMuscle::GetMetabolicPower " << m_Name << " "
        << "amhdot " << amhdot << " "
        << "slhdot " << slhdot << " "
        << "cewdot " << cewdot << " "
        << "cehdot " << cehdot << " "
        << "metabolicPower " << metabolicPower << "\n";
    }

    return metabolicPower;
}

/* the is is Umberger fortran code as supplied */
/*
c----------------------------------------------------------------------
      subroutine musmod(t)
c----------------------------------------------------------------------
      implicit double precision (a - z)
      integer i

#include "comal.inc"
#include "comusr.inc"


<<SNIP>>


c-------------------------
c--- Main muscle loop
c-------------------------
      do ii = 1,nmus


<<SNIP>>


** Note that in this code muscle excitation is "exc" and muscle
** activation is "act". "exc" cooresponds to "stim" in the article
** and "act" is the same both places

c--- Muscle activation dynamics
      act(ii) = max(act(ii),0.00001)
      adt(ii) = (exc(ii)-act(ii))*(rc1*exc(ii) + rc2)

c--- Instantaneous fiber pennation angle
      pen = asin((lceopt*sin(pen0))/lce(ii))

c--- Series elastic element force
      seext   = lmus(ii)-lce(ii)*cos(pen)-lslack
      fse(ii) = kse*max(seext,0d0)**2

c--- Parallel elastic element force
      peext   = lce(ii)-peslack*lceopt
      fpe(ii) = kpe*max(peext,0d0)**2

c--- Contractile element force
      fce(ii) = fse(ii)/cos(pen) - fpe(ii)

c--- Normalized contractile element force-length curve
      fiso(ii) = max(c0*(lce(ii)/lceopt)**2-2*c0*(lce(ii)/lceopt)+c0+1,
     1           1d-5)

c--- Contracile element velocity
c      fact = min(1d0,3.33*act(ii))  ** This is the old FACTOR
c      fact = act(ii)**0.3            ** This is my new Afactor
c      if (lce(ii) .ge. lceopt) then
c         arel=arel*fiso(ii)
c      endif
c    concentric
c      if (fce(ii) .le. (fiso(ii)*fmax*act(ii))) then
c         vce(ii) = -1*fact*lceopt*((((fiso(ii)+arel)*brel)/(fce(ii)/
c     1             (fmax*act(ii))+arel))-brel)
c newest version
        Afact = act(ii)**-0.3
        arel = Afact * arel
        if (lce(ii) .ge. lceopt) then
           arel=arel*fiso(ii)
        endif
c    concentric
        if (fce(ii) .le. (fiso(ii)*fmax*act(ii))) then
           vce(ii) = -1*lceopt*((((fiso(ii)+arel)*brel)/(fce(ii)/
       1             (fmax*act(ii))+arel))-brel)
c    eccentric
      else
         c2 = -1*fiso(ii)*fmaxecc
         c1 = (brel*fiso(ii)**2*(1-fmaxecc)**2)/(slopfac*(fiso(ii)
     1        +arel))
         c3 = -1*((brel*fiso(ii)*(1-fmaxecc))/(slopfac*(fiso(ii)
     1        +arel)))
c    hyperbolic formula
         if ((fce(ii)/(fmax*act(ii))) .le. (-1*sqrt(c1/sloplin)-c2))
     1   then
            vce(ii) = -1*lceopt*(c1/(fce(ii)/(fmax*act(ii))+c2)+c3)
         else
c    linear asymptote
            vce(ii) = lceopt*sloplin*(fce(ii)/(fmax*act(ii))+sqrt
     1                (c1/sloplin)+c2)+lceopt*(sqrt(c1*sloplin)-c3)
         endif
      endif

c--- Normalize lce and vce to optimal fiber length
c    This is used for output files only
      lcerel(ii) = lce(ii)/lceopt
      vcerel(ii) = vce(ii)/lceopt

c--- Call to metabolic energy routine
      call musenergy(ii)
      hdot(ii) = cehdot    ** This program makes use of fortran common
      wdot(ii) = cewdot    ** blocks, so cehdot and cewdot are available
                           ** after the call to musenergy()


<<SNIP>>


      enddo     ** This code is all within one big loop that is executed
                ** once for each muscle in the model

<<SNIP>>


      return
      end



c----------------------------------------------------------------------
      subroutine musenergy(ii)
c----------------------------------------------------------------------
      implicit double precision (a - z)

#include "comal.inc"
#include "comusr.inc"

c--- Parameters that are currently hard-coded but should probably
c    be read in from the .usr file!!!
      r       = 1.5
      specten = 25.0
      specten = specten*10000
      density = 1059.7
      ft      = 250.0*(arel-0.1)
      pcsa    = fmax/specten
      musmas  = pcsa*density*lceopt

c--- Caculate activation/maintenence heat rate
c    based on %FT fibers (after bolstad & ersland)
      amh = 1.28 * ft + 25.0

c--- shortening heat rate coefficient
c    4.0 x amh of st fibers at vmax of st fibers
c    0.5 x amh of ft fibers at vmax of ft fibers (barclay et al)
      vmaxst = (brel/arel) * 0.05**0.3
      vmaxft = (brel/arel)
      shst   = 4.0 *  25.0 / vmaxst
      shft   = 1.0 * 153.0 / vmaxft

c--- lengthening heat rate coefficient
c    4.0 x sh (approximates constable et al. and hawkins & mole)
      lh = 4.0 * shst

c--- Heat rate rises quickly upon excitation
c    and falls slowly upon deactivation
      if (exc(ii) .gt. act(ii)) then
         ea = exc(ii)
      else
         ea = (exc(ii) + act(ii)) * 0.5
      endif

c--- Nonlinear scaling of amhdot & slhdot
      eamh = ea**0.6
      eash = ea**2.0


** What follows is eq 18, although the nesting of the if's has
** been reversed relative to the manuscript.

c--- CE shortening
      if (vcerel(ii) .le. 0.0) then
         if (lcerel(ii) .gt. 1.0) then
            amhdot = (0.4*amh + 0.6*amh*fiso(ii))*eamh
            slhdtst = -1.0*shst*vcerel(ii)*fiso(ii)*eash
            if (slhdtst .gt. shst*vmaxst*eash) then
               slhdtst = shst*vmaxst*eash
            endif
            slhdtst = slhdtst*(1.0-ft/100.0)
            slhdtft = -1.0*(shft*vcerel(ii)*fiso(ii)*eash)*(ft/100.0)
            slhdot = slhdtst + slhdtft
         elseif (lcerel(ii) .le. 1.0) then
            amhdot = amh*eamh
            slhdtst = -1.0*shst*vcerel(ii)*eash
            if (slhdtst .gt. shst*vmaxst*eash) then
               slhdtst = shst*vmaxst*eash
            endif
            slhdtst = slhdtst*(1.0-ft/100.0)
            slhdtft = -1.0*(shft*vcerel(ii)*eash)*(ft/100.0)
            slhdot = slhdtst + slhdtft
         endif
c--- CE lengthening
      elseif (vcerel(ii) .gt. 0.0) then
         if (lcerel(ii) .gt. 1.0) then
            amhdot = (0.4*amh + 0.6*amh*fiso(ii))*eamh
            slhdot = (lh*vcerel(ii))*fiso(ii)*ea
         elseif (lcerel(ii) .le. 1.0) then
            amhdot = amh*eamh
            slhdot = (lh*vcerel(ii))*ea
         endif
      endif

c--- Account for muscle basal metabolic rate
      amhdot = max(1.0,amhdot)

c--- Sum activation, maintenece, & shortening heat rates,
c    and apply aerobic/anaerobic scaling factor
      cehdot = (amhdot + slhdot)*r

c--- CE mechanical power output, normalized to muscle mass
      cewdot = -1.0 * (vce(ii) * fce(ii) / musmas)

      return
      end
*/

void UGMMuscle::Dump()
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
            *m_DumpStream << "Time\tstim\tact\tspe\tepe\tsse\tese\tfce\tlpe\tfpe\tlse\tfse\tvce\tvse\tense\tenpe\tpse\tppe\tpce\ttension\tlength\tvelocity\tPMECH\tPMET\n";
        }
    }


    if (m_DumpStream)
    {
        *m_DumpStream << m_simulation->GetTime() << "\t" << m_stim << "\t" << m_act << "\t" << GetSPE() << "\t" <<
                GetEPE() << "\t" << GetSSE() << "\t" << GetESE() << "\t" <<
                GetFCE() << "\t" << GetLPE() << "\t" << GetFPE() << "\t" << GetLSE() << "\t" << GetFSE() << "\t" << GetVCE() << "\t" <<
                GetVSE() << "\t" << GetESE() << "\t" << GetEPE() << "\t" << GetPSE() << "\t" << GetPPE() << "\t" << GetPCE() <<
                "\t" << m_Strap->GetTension() << "\t" << m_Strap->GetLength() << "\t" << m_Strap->GetVelocity() <<
                "\t" << m_Strap->GetVelocity() * m_Strap->GetTension() << "\t" << GetMetabolicPower() <<
                "\n";
    }
}

