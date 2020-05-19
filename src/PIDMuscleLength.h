/*
 *  PIDMuscleLength.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 6/3/2011.
 *  Copyright 2011 Bill Sellers. All rights reserved.
 *
 */
#ifndef PIDMUSCLELENGTH_H
#define PIDMUSCLELENGTH_H

#include "Controller.h"

class Muscle;

class PIDMuscleLength : public Controller
{
public:
    PIDMuscleLength();

    void SetMuscle(Muscle *muscle) { m_Muscle = muscle; }
    void SetPID(double P, double I, double D) { m_Kp = P; m_Ki = I; m_Kd = D; }

    void SetNominalLength(double length) { m_nomimal_length = length; }

    virtual void SetActivation(double activation, double duration);
    virtual double GetActivation() { return m_last_activation; }

    virtual double GetValue(double time) { return 0; }; // dummy value for now

    virtual void Dump();

protected:
    Muscle *m_Muscle;
    double m_Kp;
    double m_Ki;
    double m_Kd;
    double m_setpoint;
    double m_previous_error;
    double m_error;
    double m_integral;
    double m_derivative;
    double m_output;
    double m_last_activation;
    double m_nomimal_length;
    double m_actual_position;
    double m_dt;
};

#endif // PIDMUSCLELENGTH_H
