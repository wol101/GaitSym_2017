/*
 *  NPointStrap.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 27/10/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#ifndef NPointStrap_h
#define NPointStrap_h

#include "TwoPointStrap.h"

class Body;

class NPointStrap: public TwoPointStrap
{
public:

    NPointStrap();
    virtual ~NPointStrap();

    void SetViaPoints(std::vector<Body *> *bodyList, std::vector<double *> *pointList);
    void SetViaPoints(std::vector<Body *> *bodyList, std::vector<std::string *> *pointList);

    std::vector<double *> *GetViaPoints() { return &m_ViaPointList; };
    std::vector<Body *> *GetViaPointBodies() { return &m_ViaBodyList; };

    virtual void Calculate(double deltaT);

    virtual int SanityCheck(Strap *otherStrap, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    std::vector<Body *> m_ViaBodyList;
    std::vector<double *> m_ViaPointList;
    //std::vector<dVector3 *> m_WorldViaPointList;
};


#endif
