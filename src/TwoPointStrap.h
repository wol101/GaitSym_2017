/*
 *  TwoPointStrap.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef TwoPointStrap_h
#define TwoPointStrap_h

#include "Strap.h"

class Body;

class TwoPointStrap:public Strap
{
public:

    TwoPointStrap();
    virtual ~TwoPointStrap();

    void SetOrigin(Body *body, dVector3 point);
    void SetInsertion(Body *body, dVector3 point);
    void SetOrigin(Body *body, const char *buf);
    void SetInsertion(Body *body, const char *buf);

    void GetOrigin(Body **body, double **origin) { *body = m_OriginBody; *origin = m_Origin; };
    void GetInsertion(Body **body, double **origin) { *body = m_InsertionBody; *origin = m_Insertion; };

    virtual void Calculate(double deltaT);

    virtual int SanityCheck(Strap *otherStrap, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    Body *m_OriginBody;
    dVector3 m_Origin;
    Body *m_InsertionBody;
    dVector3 m_Insertion;

};


#endif
