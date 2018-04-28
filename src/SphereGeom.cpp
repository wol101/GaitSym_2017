/*
 *  SphereGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 05/12/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "SphereGeom.h"

#ifdef USE_QT
#include "FacetedSphere.h"
#endif

SphereGeom::SphereGeom(dSpaceID space, double radius)
{
    // create the geom
    m_GeomID = dCreateSphere(space, radius);
    dGeomSetData(m_GeomID, this);
}

#ifdef USE_QT
void SphereGeom::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        m_physRep = new FacetedSphere(dGeomSphereGetRadius(m_GeomID), 4);
        m_physRep->SetColour(m_Colour);
        m_physRep->setSimulationWindow(window);
    }

    const double *bodyRotation = dBodyGetRotation(dGeomGetBody(m_GeomID));;
    const double *cylinderRelPosition = dGeomGetOffsetPosition(m_GeomID);
    const double *cylinderRelRotation = dGeomGetOffsetRotation(m_GeomID);

    dVector3 p;
    dMatrix3 r;

    // get the cylinder position in world coordinates
    dBodyGetRelPointPos(dGeomGetBody(m_GeomID), cylinderRelPosition[0],cylinderRelPosition[1], cylinderRelPosition[2], p);

    //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
    dMultiply0(r, bodyRotation, cylinderRelRotation, 3, 3, 3);

    // and draw the sphere
    m_physRep->SetColour(m_Colour);
    m_physRep->SetDisplayPosition(p[0], p[1], p[2]);
    m_physRep->SetDisplayRotation(r);
    m_physRep->Draw();

}
#endif
