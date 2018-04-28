/*
 *  SliderJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 25/05/2012.
 *  Copyright 2012 Bill Sellers. All rights reserved.
 *
 */

#ifndef SLIDERJOINT_H
#define SLIDERJOINT_H

#include "Joint.h"

class SliderJoint: public Joint
{
public:
    SliderJoint(dWorldID worldID);
    ~SliderJoint();

    void SetSliderAxis(double x, double y, double z);
    void SetSliderAxis(const char *buf);

    void SetStartDistanceReference(double startDistanceReference);
    void SetJointStops(double loStop, double hiStop);
    void SetStopCFM(double cfm);
    void SetStopERP(double erp);
    void SetStopBounce(double bounce);
    void SetStopSpringDamp(double springConstant, double dampingConstant, double integrationStep);
    void SetStopSpringERP(double springConstant, double ERP, double integrationStep);

    void GetSliderAnchor(dVector3 result);
    void GetSliderAnchor2(dVector3 result);
    void GetSliderAxis(dVector3 result);

    double GetSliderDistance();
    double GetSliderDistanceRate();

    virtual void Update();
    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    double m_StartDistanceReference;
};

#endif // SLIDERJOINT_H
