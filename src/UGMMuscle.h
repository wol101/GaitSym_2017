/*
 *  UGMMuscle.h
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

#ifndef UGMMuscle_h
#define UGMMuscle_h

#include "Muscle.h"

class Strap;

class UGMMuscle : public Muscle
{
public:

    UGMMuscle(Strap *strap);
    ~UGMMuscle();

    enum StrainModel
    {
        linear = 0,
        square
    };

    void SetStim(double stim, double timeIncrement);
    void SetFibreComposition(double fastTwitchFraction);
    void SetMuscleGeometry(double pcsa, double optimumFibreLength, double relativeWidth, double tendonLength,
                           StrainModel serialStrainModel, double serialStrainAtFmax,
                           StrainModel parallelStrainModel, double parallelStrainAtFmax);
    void SetModellingConstants(double specificTension, double relativeContractionVelocity, double muscleDensity);
    void SetAerobic(bool f) { if (f) m_s = 1.5; else m_s = 1.0; }
    void AllowReverseWork(bool f) { m_allowReverseWork = f; }

    virtual double GetMetabolicPower();
    virtual double GetElasticEnergy() { return GetESE(); }

    double GetFCE() { return m_fce; }
    double GetLPE() { return m_lce; }
    double GetFPE() { return m_Strap->GetTension() - m_fce; }
    double GetLSE() { return m_Strap->GetLength() - m_lce; }
    double GetFSE() { return m_Strap->GetTension(); }
    double GetVCE() { return m_vce; }
    double GetVPE() { return m_vce; }
    double GetVSE() { return m_Strap->GetVelocity() - m_vce; }

    double GetPSE() { return GetVSE() * - GetFSE(); } // power serial element
    double GetPPE() { return GetVPE() * - GetFPE(); } // power parallel element
    double GetPCE() { return GetVCE() * - GetFCE(); } // power contractile element
    double GetSSE() { return m_tendonlength; }
    double GetSPE() { return m_lceopt; }

    double GetESE();
    double GetEPE();

    virtual void SetActivation(double activation, double duration) { SetStim(activation, duration); }
    virtual double GetActivation() { return m_act; }

    double GetStimulation() { return m_stim; }

    virtual void Dump();

protected:

    double m_specifictension;
    double m_density;
    double m_act;
    double m_stim;
    double m_ft;
    double m_arel;
    double m_brel;
    double m_tact;
    double m_tdeact;
    double m_pcsa;
    double m_lceopt;
    double m_width;
    double m_fmax;
    double m_s;
    double m_mass;
    double m_tendonlength;
    double m_fmaxecc;
    double m_slopfac;
    double m_amh;
    double m_vmaxst;
    double m_vmaxft;
    double m_shst;
    double m_shft;
    double m_lh;
    double m_fiso;
    double m_lce;
    double m_vce;
    double m_fce;
    double m_kse;
    double m_kpe;
    StrainModel m_parallelStrainModel;
    double m_parallelStrainAtFmax;
    StrainModel m_serialStrainModel;
    double m_serialStrainAtFmax;
    bool m_allowReverseWork;
    bool m_newObject;
};

#endif



