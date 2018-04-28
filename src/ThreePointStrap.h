/*
 *  ThreePointStrap.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef ThreePointStrap_h
#define ThreePointStrap_h

#include "TwoPointStrap.h"

class Body;

class ThreePointStrap: public TwoPointStrap
{
public:

    ThreePointStrap();
    virtual ~ThreePointStrap();

    void SetMidpoint(Body *body, dVector3 point);
    void SetMidpoint(Body *body, const char *buf);

    void GetMidpoint(Body **body, double **origin) { *body = m_MidpointBody; *origin = m_Midpoint; };

    virtual void Calculate(double deltaT);

    virtual int SanityCheck(Strap *otherStrap, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    Body *m_MidpointBody;
    dVector3 m_Midpoint;
    // dVector3 m_WorldMidpoint;
};


#endif

