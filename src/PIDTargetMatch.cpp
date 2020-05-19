/*
 *  PIDTargetMatch.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 3/4/2014.
 *  Copyright 2014 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "PIDTargetMatch.h"
#include "Muscle.h"
#include "DataTarget.h"

PIDTargetMatch::PIDTargetMatch()
{
    m_Muscle = 0;
    m_Target = 0;
    m_Kp = 0;
    m_Ki = 0;
    m_Kd = 0;
    m_previous_error = 0;
    m_error = 0;
    m_integral = 0;
    m_derivative = 0;
    m_output = 0;
    m_last_activation = 0;
    m_last_set_activation = 0;
    m_dt = 0;
}

// the activation value here is just a global gain modifier
// probably it should be set to 1 for normal use
// and 0 to deactivate the controller (although this will just leave the current value fixed)
void PIDTargetMatch::SetActivation(double activation, double duration)
{
    if (activation != m_last_set_activation) // reset the error values when the target value changes
    {
        m_last_set_activation = activation;
        m_previous_error = 0;
        m_integral = 0;
    }
    m_dt = duration;

    // do the PID calculations
    m_error = m_Target->GetError(0); // this assumes that the error is at index zero and it won't always be
    m_integral = m_integral + (m_error * m_dt);
    m_derivative = (m_error - m_previous_error) / m_dt;
    m_output = (m_Kp * m_error) + (m_Ki * m_integral) + (m_Kd * m_derivative);
    m_previous_error = m_error;

    // now alter the activation based on the PID output
    double muscleActivation = m_last_activation - activation * m_output;
    if (muscleActivation < 0) muscleActivation = 0;
    else if (muscleActivation > 1) muscleActivation = 1;

#ifdef LOCAL_DEBUG
    if (m_Name == "RightAdductorBrevisRightHipJointDataTargetController")
    {
        std::cerr << "error " << error << " integral " << integral << " derivative " << derivative << " output " << output <<
                     " muscleActivation " << muscleActivation << "\n";
    }
#endif

    std::cerr << "PIDTargetMatch::SetActivation currently untested\n";
    // m_Muscle->SetActivation(muscleActivation, duration);
    m_last_activation = muscleActivation;
}

void PIDTargetMatch::Dump()
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
            *m_DumpStream << "Time\tKp\tKi\tKd\tprevious_error\terror\tintegral\tderivative\toutput\tdt\tlast_set_activation\tlast_activation\n";
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
                         "\t" << m_last_set_activation <<
                         "\t" << m_last_activation <<
                         "\n";
    }
}
