/*
 *  Environment.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Mar 26 2005.
 *  Copyright (c) 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Environment_h
#define Environment_h

#include "NamedObject.h"

#include <vector>

#ifdef USE_QT
class SimulationWindow;
namespace irr { namespace scene { class IMeshSceneNode; } }
#endif

class Geom;

class Environment: public NamedObject
{
public:
    Environment();
    ~Environment();

    void AddGeom(Geom *geom);
    Geom *GetGeom(int i) { return m_GeomList[i]; }

    virtual void WriteToXMLStream(std::ostream &outputStream);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    std::vector<Geom *> m_GeomList;

#ifdef USE_QT
    irr::scene::IMeshSceneNode *m_axisSceneNode;
#endif

};


#endif



