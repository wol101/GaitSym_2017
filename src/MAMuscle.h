/*
 *  MAMuscle.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// MAMuscle - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a parallel spring element

#ifndef MAMuscle_h
#define MAMuscle_h

#include "Muscle.h"

class Strap;

class MAMuscle : public Muscle
{
public:

    MAMuscle(Strap *strap);
    ~MAMuscle();

    void SetVMax(double vMax) { m_VMax = vMax; }
    void SetF0(double f0) { m_F0 = f0; }
    void SetK(double k) { m_K = k; }

    virtual double GetMetabolicPower();

    virtual void SetActivation(double activation, double /* duration */) { SetAlpha(activation); }
    virtual double GetActivation() { return m_Alpha; }
    virtual double GetElasticEnergy() { return 0; }

    virtual void Dump();

protected:

    void SetAlpha(double alpha);

    double m_VMax;
    double m_F0;
    double m_K;
    double m_Alpha;
};








#endif // MAMuscle_h
