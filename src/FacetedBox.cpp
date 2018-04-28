/*
 *  FacetedBox.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 31/05/2012.
 *  Copyright 2012 Bill Sellers. All rights reserved.
 *
 */

#ifdef USE_QT

#include <ode/ode.h>
#include <vector>

#include "FacetedBox.h"
#include "PGDMath.h"

// draw a box of dimensions lx, ly, lz with origin at the centre

FacetedBox::FacetedBox(double lx, double ly, double lz)
{
    lx *= 0.5;
    ly *= 0.5;
    lz *= 0.5;

    std::vector<pgd::Vector> triangleStrip;

    /* drawstuff box code
    // sides
    glBegin (GL_TRIANGLE_STRIP);
    glNormal3f (-1,0,0);
    glVertex3f (-lx,-ly,-lz);
    glVertex3f (-lx,-ly,lz);
    glVertex3f (-lx,ly,-lz);
    glVertex3f (-lx,ly,lz);
    glNormal3f (0,1,0);
    glVertex3f (lx,ly,-lz);
    glVertex3f (lx,ly,lz);
    glNormal3f (1,0,0);
    glVertex3f (lx,-ly,-lz);
    glVertex3f (lx,-ly,lz);
    glNormal3f (0,-1,0);
    glVertex3f (-lx,-ly,-lz);
    glVertex3f (-lx,-ly,lz);
    glEnd();

    // top face
    glBegin (GL_TRIANGLE_FAN);
    glNormal3f (0,0,1);
    glVertex3f (-lx,-ly,lz);
    glVertex3f (lx,-ly,lz);
    glVertex3f (lx,ly,lz);
    glVertex3f (-lx,ly,lz);
    glEnd();

    // bottom face
    glBegin (GL_TRIANGLE_FAN);
    glNormal3f (0,0,-1);
    glVertex3f (-lx,-ly,-lz);
    glVertex3f (-lx,ly,-lz);
    glVertex3f (lx,ly,-lz);
    glVertex3f (lx,-ly,-lz);
    glEnd();
    */

    // sides
    triangleStrip.push_back(pgd::Vector(-lx,-ly,-lz));
    triangleStrip.push_back(pgd::Vector(-lx,-ly,lz));
    triangleStrip.push_back(pgd::Vector(-lx,ly,-lz));
    triangleStrip.push_back(pgd::Vector(-lx,ly,lz));
    triangleStrip.push_back(pgd::Vector(lx,ly,-lz));
    triangleStrip.push_back(pgd::Vector(lx,ly,lz));
    triangleStrip.push_back(pgd::Vector(lx,-ly,-lz));
    triangleStrip.push_back(pgd::Vector(lx,-ly,lz));
    triangleStrip.push_back(pgd::Vector(-lx,-ly,-lz));
    triangleStrip.push_back(pgd::Vector(-lx,-ly,lz));
    AddTriangleStrip(triangleStrip);

    // top face
    triangleStrip.clear();
    triangleStrip.push_back(pgd::Vector(-lx,-ly,lz));
    triangleStrip.push_back(pgd::Vector(lx,-ly,lz));
    triangleStrip.push_back(pgd::Vector(lx,ly,lz));
    triangleStrip.push_back(pgd::Vector(-lx,ly,lz));
    AddTriangleFan(triangleStrip);

    // bottom face
    triangleStrip.clear();
    triangleStrip.push_back(pgd::Vector(-lx,-ly,-lz));
    triangleStrip.push_back(pgd::Vector(-lx,ly,-lz));
    triangleStrip.push_back(pgd::Vector(lx,ly,-lz));
    triangleStrip.push_back(pgd::Vector(lx,-ly,-lz));
    AddTriangleFan(triangleStrip);

}

void FacetedBox::AddTriangleStrip(std::vector<pgd::Vector> &triangleStrip)
{
    double triangle[9];
    unsigned int i;

    for (i = 2; i < triangleStrip.size(); i++)
    {
        if (i % 2 == 0)
        {
            triangle[0] = triangleStrip[i - 2].x;
            triangle[1] = triangleStrip[i - 2].y;
            triangle[2] = triangleStrip[i - 2].z;
            triangle[3] = triangleStrip[i - 1].x;
            triangle[4] = triangleStrip[i - 1].y;
            triangle[5] = triangleStrip[i - 1].z;
            triangle[6] = triangleStrip[i].x;
            triangle[7] = triangleStrip[i].y;
            triangle[8] = triangleStrip[i].z;
        }
        else
        {
            triangle[0] = triangleStrip[i - 2].x;
            triangle[1] = triangleStrip[i - 2].y;
            triangle[2] = triangleStrip[i - 2].z;
            triangle[6] = triangleStrip[i - 1].x;
            triangle[7] = triangleStrip[i - 1].y;
            triangle[8] = triangleStrip[i - 1].z;
            triangle[3] = triangleStrip[i].x;
            triangle[4] = triangleStrip[i].y;
            triangle[5] = triangleStrip[i].z;
        }
        AddTriangle(triangle);
    }
}

void FacetedBox::AddTriangleFan(std::vector<pgd::Vector> &triangleFan)
{
    double triangle[9];
    unsigned int i;

    triangle[0] = triangleFan[0].x;
    triangle[1] = triangleFan[0].y;
    triangle[2] = triangleFan[0].z;
    for (i = 2; i < triangleFan.size(); i++)
    {
        triangle[3] = triangleFan[i - 1].x;
        triangle[4] = triangleFan[i - 1].y;
        triangle[5] = triangleFan[i - 1].z;
        triangle[6] = triangleFan[i].x;
        triangle[7] = triangleFan[i].y;
        triangle[8] = triangleFan[i].z;
        AddTriangle(triangle);
    }
}

#endif
