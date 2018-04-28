/*
 *  ButterworthFilter.h
 *  GaitSym2016
 *
 *  Created by Bill Sellers on Sat Jun 4 2016.
 *  Copyright (c) 2016 Bill Sellers. All rights reserved.
 *
 *  Implements a 2nd Order Butterworth Low Pass IIR Filter
 *
 */

#ifndef BUTTERWORTHFILTER_H
#define BUTTERWORTHFILTER_H

#include "Filter.h"

class ButterworthFilter : Filter
{
public:
    ButterworthFilter();
    ButterworthFilter(double cutoffFrequency, double samplingFrequency);

    virtual void AddNewSample(double x);
    virtual double Output();

    void CalculateCoefficients(double cutoffFrequency, double samplingFrequency);

    double xn() const;
    double yn() const;

    double cutoffFrequency() const;
    double samplingFrequency() const;

protected:

    double m_cutoffFrequency;
    double m_samplingFrequency;
    double m_b0;
    double m_b1;
    double m_b2;
    double m_a1;
    double m_a2;
//    double m_xn; // use m_xn from the base class
    double m_xnminus1;
    double m_xnminus2;
    double m_yn;
    double m_ynminus1;
    double m_ynminus2;
};

class SharedButterworthFilter : public Filter
{
public:
    SharedButterworthFilter();
    SharedButterworthFilter(double cutoffFrequency, double samplingFrequency);

    virtual void AddNewSample(double x);
    virtual double Output();

    static void CalculateCoefficients(double cutoffFrequency, double samplingFrequency);

    double xn() const;
    double yn() const;

    double cutoffFrequency() const;
    double samplingFrequency() const;

protected:

    static double m_cutoffFrequency;
    static double m_samplingFrequency;
    static double m_b0;
    static double m_b1;
    static double m_b2;
    static double m_a1;
    static double m_a2;
//    double m_xn; // use m_xn from the base class
    double m_xnminus1;
    double m_xnminus2;
    double m_yn;
    double m_ynminus1;
    double m_ynminus2;
};

#endif // BUTTERWORTHFILTER_H
