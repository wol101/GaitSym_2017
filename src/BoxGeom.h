/*
 *  BoxGeom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 30/05/2012.
 *  Copyright 2012 Bill Sellers. All rights reserved.
 *
 */

#ifndef BOXGEOM_H
#define BOXGEOM_H

#include "Geom.h"

class BoxGeom : public Geom
{
public:
    BoxGeom(dSpaceID space, double lx, double ly, double lz);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

};


#endif // BOXGEOM_H
