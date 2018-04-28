/*
 *  TrimeshGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 11/08/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "TrimeshGeom.h"
#include "FacetedObject.h"

// create the trimesh object
// note FacetedObject is used for drawing so must remain valid
// if drawing is required. However it isn't used for colision detection after creation
// and it isn't deleted by the TrimeshGeom
TrimeshGeom::TrimeshGeom(dSpaceID space, FacetedObject *facetedObject)
{
    /*
    facetedObject->CalculateTrimesh(&m_Vertices, &m_NumVertices, &m_VertexStride, &m_TriIndexes, &m_NumTriIndexes, &m_TriStride);
    m_TriMeshDataID = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle (m_TriMeshDataID, m_Vertices, m_VertexStride, m_NumVertices, m_TriIndexes, m_NumTriIndexes, m_TriStride);
    */

    m_Vertices = facetedObject->GetVertexList();
    m_TriIndexes = facetedObject->GetIndexList();
    m_NumVertices = facetedObject->GetNumVertices();
    m_NumTriIndexes = m_NumVertices;

    m_TriMeshDataID = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildDouble(m_TriMeshDataID,
                                m_Vertices, 3 * sizeof(double), m_NumVertices,
                                m_TriIndexes, m_NumTriIndexes, 3 * sizeof(int));

    m_GeomID =  dCreateTriMesh (space, m_TriMeshDataID, 0, 0, 0);
    dGeomSetData(m_GeomID, this);

    // and finally assign the faceted object
    m_FacetedObject = facetedObject;
}

TrimeshGeom::~TrimeshGeom()
{
    dGeomTriMeshDataDestroy(m_TriMeshDataID);
    //delete [] m_Vertices;
    //delete [] m_TriIndexes;
}

#ifdef USE_QT
void TrimeshGeom::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    m_FacetedObject->setSimulationWindow(window);
    m_FacetedObject->Draw();
}
#endif
