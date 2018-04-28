/*
 *  CappedCylinderGeom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef CappedCylinderGeom_h
#define CappedCylinderGeom_h

#include "Geom.h"

class CappedCylinderGeom:public Geom
{
public:

    CappedCylinderGeom(dSpaceID space, double radius, double length);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif
};


#endif

