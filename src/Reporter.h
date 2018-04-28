/*
 *  Reporter.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 06/01/2010.
 *  Copyright 2010 Bill Sellers. All rights reserved.
 *
 */

#ifndef REPORTER_H
#define REPORTER_H

#include "NamedObject.h"

class SimulationWindow;

class Reporter : public NamedObject
{
public:
    Reporter();

    virtual bool ShouldAbort() { return false; }

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window) = 0;
#endif
};

#endif // REPORTER_H
