/*
 *  Face.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */
/*
#include <string.h>

#include <ode/ode.h>

#include "Face.h"

Face::Face()
{
    mNumVertices = 0;
    mVertexList = 0;
    mNormal[0] = 1;
    mNormal[1] = 0;
    mNormal[2] = 0;
}

Face::~Face()
{
    if (mVertexList) delete [] mVertexList;
}

void Face::SetNumVertices(int numVertices)
{
    if (numVertices != mNumVertices)
    {
        mNumVertices = numVertices;
        if (mVertexList) delete [] mVertexList;
        mVertexList = new int[mNumVertices];
    }
}

Face& Face::operator=(Face &in)
{
    SetNumVertices(in.GetNumVertices());
    for (int i = 0; i < mNumVertices; i++)
        mVertexList[i] = in.GetVertex(i);

    in.GetNormal(mNormal);

    return *this;
}

void Face::ReverseVertexOrder()
{
    int tmp;
    int *p1 = mVertexList;
    int *p2 = mVertexList + mNumVertices - 1;

    while (p1 < p2)
    {
        tmp = *p1;
        *p1 = *p2;
        *p2 = tmp;
        p1++;
        p2--;
    }
}

void Face::OffsetVertices(int offset)
{
    int i;
    for (i = 0; i < mNumVertices; i++) mVertexList[i] += offset;
}
*/

