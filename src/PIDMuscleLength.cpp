/*
 *  PIDMuscleLength.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 6/3/2011.
 *  Copyright 2011 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "PIDMuscleLength.h"
#include "Muscle.h"

PIDMuscleLength::PIDMuscleLength()
{
    m_Muscle = 0;
    m_Kp = 0;
    m_Ki = 0;
    m_Kd = 0;
    m_setpoint = 0;
    m_previous_error = 0;
    m_error = 0;
    m_integral = 0;
    m_derivative = 0;
    m_output = 0;
    m_last_activation = 0;
    m_nomimal_length = 1;
    m_actual_position = 0;
    m_dt = 0;
}

void PIDMuscleLength::SetActivation(double activation, double duration)
{
    if (activation != m_setpoint) // reset the error values when the target value changes
    {
        m_setpoint = activation;
        m_previous_error = 0;
        m_integral = 0;
    }
    m_actual_position = m_Muscle->GetLength() / m_nomimal_length;
    m_dt = duration;

    // do the PID calculations
    m_error = m_setpoint - m_actual_position;
    m_integral = m_integral + (m_error * m_dt);
    m_derivative = (m_error - m_previous_error) / m_dt;
    m_output = (m_Kp * m_error) + (m_Ki * m_integral) + (m_Kd * m_derivative);
    m_previous_error = m_error;

    // now alter the activation based on the PID output
    activation = m_last_activation - m_output;
    if (activation < 0) activation = 0;
    else if (activation > 1) activation = 1;

    std::cerr << "PIDMuscleLength::SetActivation currently untested\n";
    // m_Muscle->SetActivation(activation, duration);
    m_last_activation = activation;
}

void PIDMuscleLength::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == nullptr)
        {
            if (m_Name.size() == 0) std::cerr << "NamedObject::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tKp\tKi\tKd\tprevious_error\terror\tintegral\tderivative\toutput\tdt\tactual_position\tsetpoint\n";
        }
    }

    if (m_DumpStream)
    {
        *m_DumpStream << m_simulation->GetTime() <<
                         "\t" << m_Kp <<
                         "\t" << m_Ki <<
                         "\t" << m_Kd <<
                         "\t" << m_previous_error <<
                         "\t" << m_error <<
                         "\t" << m_integral <<
                         "\t" << m_derivative <<
                         "\t" << m_output <<
                         "\t" << m_dt <<
                         "\t" << m_actual_position <<
                         "\t" << m_setpoint <<
                         "\n";
    }
}

