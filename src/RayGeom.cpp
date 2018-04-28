/*
 *  RayGeom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 14/08/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "RayGeom.h"

// creates the ray geometry with defined
// length
// origin x, y, z
// direction dx, dy, dz
RayGeom::RayGeom(dSpaceID space, double length, double x, double y, double z, double dx, double dy, double dz)
{
    // create the geom
    m_GeomID = dCreateRay(space, length);
    dGeomRaySet(m_GeomID, x, y, z, dx, dy, dz);
    dGeomSetData(m_GeomID, this);
}

// set some ray control parameters
void RayGeom::SetParams(int firstContact, int backfaceCull, int closestHit)
{
#ifdef USE_OLD_ODE
    dGeomRaySetParams(m_GeomID, firstContact, backfaceCull);
#else
    dGeomRaySetFirstContact (m_GeomID, firstContact);
    dGeomRaySetBackfaceCull (m_GeomID, backfaceCull);
#endif
    dGeomRaySetClosestHit (m_GeomID, closestHit);
}

#ifdef USE_QT
void RayGeom::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    // draw ray here
    double length;
    dVector3 start;
    dVector3 dir;
    length = dGeomRayGetLength (m_GeomID);
    dGeomRayGet(m_GeomID, start, dir);
/*
    glDisable(GL_LIGHTING);

    // this assumes that the values are in world coordinates
    // which is true if there is no associated body
    glBegin(GL_LINES);
    glColor4f(m_Colour.r, m_Colour.g, m_Colour.b, m_Colour.alpha);
    glVertex3f(start[0], start[1], start[2]);
    glVertex3f(start[0] + length * dir[0], start[1] + length * dir[1], start[2] + length * dir[2]);
    glEnd();

    glEnable(GL_LIGHTING);
*/
}
#endif
