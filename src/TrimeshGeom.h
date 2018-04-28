/*
 *  TrimeshGeom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 11/08/2009.
 *  Copyright 2009 Bill Sellers. All rights reserved.
 *
 */

#ifndef TRIMESHGEOM_H
#define TRIMESHGEOM_H

#include "Geom.h"

class FacetedObject;
class StridedVertex;
class StridedTri;
class GimpactStridedVertex;

class TrimeshGeom : public Geom
{
public:
    TrimeshGeom(dSpaceID space, FacetedObject *facetedObject);
    virtual ~TrimeshGeom();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
#endif

protected:

    FacetedObject *m_FacetedObject;

#ifdef USE_GIMPACT
    GimpactStridedVertex *m_Vertices;
#else
    double *m_Vertices;
#endif
    int m_NumVertices;
    int m_VertexStride;
    int *m_TriIndexes;
    int m_NumTriIndexes;
    int m_TriStride;
    dTriMeshDataID m_TriMeshDataID;

};

#endif // TRIMESHGEOM_H

