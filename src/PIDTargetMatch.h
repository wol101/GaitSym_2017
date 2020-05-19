/*
 *  PIDTargetMatch.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 3/5/2014.
 *  Copyright 2014 Bill Sellers. All rights reserved.
 *
 */

#ifndef PIDTARGETMATCH_H
#define PIDTARGETMATCH_H

#include "Controller.h"

class Muscle;
class DataTarget;

class PIDTargetMatch : public Controller
{
public:
    PIDTargetMatch();

    void SetMuscle(Muscle *muscle) { m_Muscle = muscle; }
    void SetPID(double P, double I, double D) { m_Kp = P; m_Ki = I; m_Kd = D; }
    void SetDataTarget(DataTarget *target) { m_Target = target; }

    virtual void SetActivation(double activation, double duration);
    virtual double GetActivation() { return m_last_activation; }

    virtual double GetValue(double time) { return 0; }; // dummy value for now

    virtual void Dump();

protected:
    Muscle *m_Muscle;
    DataTarget *m_Target;

    double m_Kp;
    double m_Ki;
    double m_Kd;
    double m_previous_error;
    double m_error;
    double m_integral;
    double m_derivative;
    double m_output;
    double m_last_activation;
    double m_last_set_activation;
    double m_dt;
};

#endif // PIDTARGETMATCH_H
