/*
 *  MovingAverage.h
 *  GaitSym2016
 *
 *  Created by Bill Sellers on Sat Jun 4 2016.
 *  Copyright (c) 2016 Bill Sellers. All rights reserved.
 *
 *  Implements a Moving Average FIR Filter
 *
 */

#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H

#include "Filter.h"

class MovingAverage : public Filter
{
public:
    MovingAverage();
    MovingAverage(int window);
    ~MovingAverage();

    virtual void AddNewSample(double x);
    virtual double Output();

    void InitialiseBuffer(int window);

    double sum() const;
    double average() const;
    int window() const;

protected:
    int m_window;
    int m_index;
    double *m_buffer;
    double m_sum;
    double m_average;
};

#endif // MOVINGAVERAGE_H
