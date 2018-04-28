/*
 *  HingeJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef HingeJoint_h
#define HingeJoint_h

#include "Joint.h"

class HingeJoint: public Joint
{
public:

    HingeJoint(dWorldID worldID);
    ~HingeJoint();

    void SetHingeAnchor (double x, double y, double z);
    void SetHingeAxis(double x, double y, double z);
    void SetHingeAnchor (const char *buf);
    void SetHingeAxis(const char *buf);

    void SetStartAngleReference(double startAngleReference);
    void SetJointStops(double loStop, double hiStop);
    void SetStopCFM(double cfm);
    void SetStopERP(double erp);
    void SetStopBounce(double bounce);
    void SetStopSpringDamp(double springConstant, double dampingConstant, double integrationStep);
    void SetStopSpringERP(double springConstant, double ERP, double integrationStep);

    void GetHingeAnchor(dVector3 result);
    void GetHingeAnchor2(dVector3 result);
    void GetHingeAxis(dVector3 result);

    double GetHingeAngle();
    double GetHingeAngleRate();

    void SetTorqueLimits(double loStopTorqueLimit, double hiStopTorqueLimit)
    {
        m_LoStopTorqueLimit = loStopTorqueLimit;
        m_HiStopTorqueLimit = hiStopTorqueLimit;
    }
    int TestLimits();
    void SetStopTorqueWindow(int window);

    virtual void Update();
    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    void CalculateStopTorque();

    double m_StartAngleReference;

    double m_HiStopTorqueLimit;
    double m_LoStopTorqueLimit;
    double m_axisTorque;

    double *m_axisTorqueList;
    double m_axisTorqueTotal;
    double m_axisTorqueMean;
    int m_axisTorqueIndex;
    int m_axisTorqueWindow;

};



#endif
