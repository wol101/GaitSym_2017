/*
 *  PIDErrorInController.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 08/01/2017.
 *  Copyright 2017 Bill Sellers. All rights reserved.
 *
 */

#include "PIDErrorInController.h"
#include "Simulation.h"

#include <fstream>

#define RANGE(x, l, h) if (x < (l)) x = (l); if (x > (h)) x = (h);

PIDErrorInController::PIDErrorInController()
{
    // these are the standard PID controller values
    m_Kp = 0;
    m_Ki = 0;
    m_Kd = 0;
    m_previous_error = 0;
    m_error = 0;
    m_integral = 0;
    m_derivative = 0;
    m_output = 0;
    m_dt = 0;
//    m_outputIsDelta = false;
}

void PIDErrorInController::Initialise(double Kp, double Ki, double Kd)
{
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
    m_previous_error = 0;
    m_error = 0;
    m_integral = 0;
    m_derivative = 0;
    m_output = 0;
    m_LastTime = 0;
    m_LastValue = 0;
    m_dt = 0;
}

double PIDErrorInController::GetValue(double time)
{
    if (time == m_LastTime) return m_LastValue;
    m_dt = time - m_LastTime;
    m_LastTime = time;

    // in this driver, the error is driven by the upstream driver
    m_error = SumDrivers(time);

    // do the PID calculations
    m_integral = m_integral + (m_error * m_dt);
    m_derivative = (m_error - m_previous_error) / m_dt;
    m_output = (m_Kp * m_error) + (m_Ki * m_integral) + (m_Kd * m_derivative);
    m_previous_error = m_error;

    // now alter the output based on the PID output
    double output = m_LastValue - m_output;
    RANGE(output, m_MinValue, m_MaxValue);
    m_LastValue = output;
    return output;
}

//void PIDErrorInController::setOutputIsDelta(bool outputIsDelta)
//{
//    m_outputIsDelta = outputIsDelta;
//}

void PIDErrorInController::Dump()
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
            *m_DumpStream << "Time\tKp\tKi\tKd\tprevious_error\terror\tintegral\tderivative\toutput\tdt\n";
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
                         "\n";
    }
}

