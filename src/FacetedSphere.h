/*
 *  FacetedSphere.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 03/01/2006.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

#ifndef FacetedSphere_h
#define FacetedSphere_h

#include <sstream>

#include "FacetedObject.h"

class FacetedSphere: public FacetedObject
{
public:
    FacetedSphere(double radius, int level);

    virtual void WritePOVRay(std::ostringstream &theString);

protected:
    int m_Level;
    double m_Radius;
};

#endif
