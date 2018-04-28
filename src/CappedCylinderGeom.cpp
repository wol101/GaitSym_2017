/*
 *  CappedCylinderGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include "CappedCylinderGeom.h"

#ifdef USE_QT
#include "FacetedCappedCylinder.h"
#endif

CappedCylinderGeom::CappedCylinderGeom(dSpaceID space, double radius, double length)
{
    // create the geom
    m_GeomID = dCreateCapsule(space, radius, length);

    dGeomSetData(m_GeomID, this);
}

#ifdef USE_QT
void CappedCylinderGeom::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        // get the length and radius
        double radius, length;
        dGeomCapsuleGetParams (m_GeomID, &radius, &length);

        int capped_cylinder_quality = 32;
        m_physRep = new FacetedCappedCylinder(length, radius, capped_cylinder_quality);
        m_physRep->SetColour(m_Colour);
        m_physRep->setSimulationWindow(window);
    }

    const double *bodyRotation = dBodyGetRotation(dGeomGetBody(m_GeomID));
    const double *cylinderRelPosition = dGeomGetOffsetPosition(m_GeomID);
    const double *cylinderRelRotation = dGeomGetOffsetRotation(m_GeomID);

    dVector3 p;
    dMatrix3 r;

    // get the cylinder position in world coordinates
    dBodyGetRelPointPos(dGeomGetBody(m_GeomID), cylinderRelPosition[0],cylinderRelPosition[1], cylinderRelPosition[2], p);

    //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
    dMultiply0(r, bodyRotation, cylinderRelRotation, 3, 3, 3);

    // and draw the capped cylinder
    m_physRep->SetColour(m_Colour);
    m_physRep->SetDisplayRotation(r);
    m_physRep->SetDisplayPosition(p[0], p[1], p[2]);
    m_physRep->Draw();
}
#endif
