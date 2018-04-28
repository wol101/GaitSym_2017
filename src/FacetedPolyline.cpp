/*
 *  FacetedPolyline.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 10/08/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

// Uses algorithms described in Geometric Tools for Computer Graphics, Scheider & Eberly 2003

#ifdef USE_QT

#include <ode/ode.h>
#include <iostream>
#include <cmath>

#include "FacetedPolyline.h"
#include "Face.h"

FacetedPolyline::FacetedPolyline(std::vector<pgd::Vector> *polyline, double radius, int n)
{
    std::vector<pgd::Vector> profile;

    // need to add extra tails to the polyline for direction padding
    std::vector<pgd::Vector> newPolyline;
    pgd::Vector v0 = (*polyline)[1] - (*polyline)[0];
    pgd::Vector v1 = (*polyline)[0] - v0;
    newPolyline.push_back(v1);
    for (unsigned int i = 0; i < polyline->size(); i++) newPolyline.push_back((*polyline)[i]);
    v0 = (*polyline)[polyline->size() - 1] - (*polyline)[polyline->size() - 2];
    v1 = (*polyline)[polyline->size() - 1] + v0;
    newPolyline.push_back(v1);

    // create the profile
    double delTheta = 2 * M_PI / n;
    double theta = M_PI / 2;
    for (int i = 0; i < n; i++)
    {
        v0.x = cos(theta) * radius;
        v0.y = sin(theta) * radius;
        v0.z = 0;
        theta -= delTheta;
        profile.push_back(v0);
    }

    Extrude(&newPolyline, &profile);

}

/* this was an attempt at optimisation because the AddFace stage is slow
   however it doesn't help much. I think it might be worth reworking the
   core FacetedObject class so that it always stores triangle which might
   make it much quicker (especially since I think only triangles work with
   the new VBO based design).
   */
/*
FacetedPolyline::FacetedPolyline(std::vector<pgd::Vector> *polyline, double radius, int n)
{
    unsigned int i, j;
    std::vector<pgd::Vector> profile;
    std::vector<FPPolygon *> faces;

    //QElapsedTimer timer;
    //timer.start();

    // need to add extra tails to the polyline for direction padding
    std::vector<pgd::Vector> newPolyline;
    pgd::Vector v0 = (*polyline)[1] - (*polyline)[0];
    pgd::Vector v1 = (*polyline)[0] - v0;
    newPolyline.push_back(v1);
    for (i = 0; i < polyline->size(); i++) newPolyline.push_back((*polyline)[i]);
    v0 = (*polyline)[polyline->size() - 1] - (*polyline)[polyline->size() - 2];
    v1 = (*polyline)[polyline->size() - 1] + v0;
    newPolyline.push_back(v1);

    //std::cerr << "Polyline copy: " << timer.restart () << "\n";

    // create the profile
    double delTheta = 2 * M_PI / n;
    double theta = M_PI / 2;
    for (i = 0; i < n; i++)
    {
        v0.x = cos(theta) * radius;
        v0.y = sin(theta) * radius;
        v0.z = 0;
        theta -= delTheta;
        profile.push_back(v0);
    }

    //std::cerr << "Profile generation: " << timer.restart () << "\n";

    Extrude(&newPolyline, &profile, &faces);

    //std::cerr << "Extrusion: " << timer.restart () << "\n";

    // allocate vertices and triangle
    mNumFacesAllocated = (faces.size() - 2) * 2 + (n - 2) * 2;
    mFaceList = new Face *[mNumFacesAllocated];
    mNumVerticesAllocated = mNumFacesAllocated * 3;
    mVertexList = new Vertex *[mNumVerticesAllocated];


    // add faces as triangles
    Vertex triangle[3];
    Face *face;
    Vertex *vertex;
    for (i = 0; i < faces.size(); i++)
    {
        triangle[0].x = faces[i]->vertices[0].x;
        triangle[0].y = faces[i]->vertices[0].y;
        triangle[0].z = faces[i]->vertices[0].z;
        for (j = 2; j < faces[i]->vertices.size(); j++)
        {
            triangle[1].x = faces[i]->vertices[j - 1].x;
            triangle[1].y = faces[i]->vertices[j - 1].y;
            triangle[1].z = faces[i]->vertices[j - 1].z;
            triangle[2].x = faces[i]->vertices[j].x;
            triangle[2].y = faces[i]->vertices[j].y;
            triangle[2].z = faces[i]->vertices[j].z;
            face = new Face();
            face->SetNumVertices(3);
            vertex = new Vertex();
            *vertex = triangle[0];
            mVertexList[mNumVertices] = vertex;
            face->SetVertex(0, mNumVertices);
            mNumVertices++;
            vertex = new Vertex();
            *vertex = triangle[1];
            mVertexList[mNumVertices] = vertex;
            face->SetVertex(1, mNumVertices);
            mNumVertices++;
            vertex = new Vertex();
            *vertex = triangle[2];
            mVertexList[mNumVertices] = vertex;
            face->SetVertex(2, mNumVertices);
            mNumVertices++;
            mFaceList[mNumFaces] = face;
            mNumFaces++;
        }
    }

    //std::cerr << "Adding faces: " << timer.restart () << "\n";

    SetDrawClockwise(false);
    CalculateNormals();

    //std::cerr << "Normal generation: " << timer.elapsed () << "\n";

    for (i = 0; i < faces.size(); i++) delete faces[i];

}
*/

// extrude profile along a poly line using sharp corners
// profile is a 2D shape with z = 0 for all values.
// polyline needs to have no parallel neighbouring segements
// anti-clockwise winding assumed (I think)
// first and last point of polyline used for direction only!
void FacetedPolyline::Extrude(std::vector<pgd::Vector> *polyline, std::vector<pgd::Vector> *profile)
{
    unsigned int i, j;
    Line3D line;
    pgd::Vector v1, v2, p1, p2;
    double epsilon = 0.000001;
    double *polygon = new double[profile->size() * 3];


    // define the planes of the joins
    std::vector<Plane3D> joinPlanes;
    Plane3D plane;
    for (i = 1; i < (*polyline).size() - 1; i++)
    {
        v1 = (*polyline)[i] - (*polyline)[i - 1];
        v2 = (*polyline)[i + 1] - (*polyline)[i];
        v1.Normalize();
        v2.Normalize();
        p1 = v1 - v2;
        if (p1.Magnitude() > epsilon) // not parallel so use two vector form of plane
        {
            p2 = v1 ^ v2;
            plane = Plane3D(&(*polyline)[i], &p1, &p2);
        }
        else // parallel so use normal-point form
        {
            plane = Plane3D(&(*polyline)[i], &v1);
        }

        joinPlanes.push_back(plane);
    }

    // define the rotated (*profile) for the first line segment
    // note for a truly generic routine you need two rotations to allow an up vector to be defined
    pgd::Vector zVec(0, 0, 1);
    v1 = (*polyline)[1] - (*polyline)[0];
    pgd::Quaternion q = pgd::FindRotation(zVec, v1);
    std::vector<pgd::Vector> rotatedProfile;
    for (i = 0; i < (*profile).size(); i++)
    {
        pgd::Vector v = pgd::QVRotate(q, (*profile)[i]);
        rotatedProfile.push_back(v);
    }

    // find the intersections on the join planes
    std::vector<pgd::Vector> vertexList;
    for (i = 0; i < rotatedProfile.size(); i++)
    {
        v2 = (*polyline)[0] + rotatedProfile[i];
        line = Line3D(&v2, &v1);
        for (j = 0; j < joinPlanes.size(); j++)
        {
            if (Intersection(&line, &joinPlanes[j], &v2))
            {
                vertexList.push_back(v2);
                if (j < joinPlanes.size() - 1)
                {
                    p1 = (*polyline)[j + 2] - (*polyline)[j + 1];
                    line = Line3D(&v2, &p1);
                }
            }
            else
            {
                std::cerr << "Error finding line & plane intersection" << std::endl;
                return;
            }
        }
    }

    // now construct faces
    for (i = 0; i < rotatedProfile.size(); i++)
    {
        for (j = 0; j < joinPlanes.size() - 1; j++)
        {
            if (i < rotatedProfile.size() - 1)
            {
                //face->vertices.push_back(vertexList[j + joinPlanes.size() * i]);
                //face->vertices.push_back(vertexList[1 + j + joinPlanes.size() * i]);
                //face->vertices.push_back(vertexList[1 + j + joinPlanes.size() * (i + 1)]);
                //face->vertices.push_back(vertexList[j + joinPlanes.size() * (i + 1)]);
                polygon[0] = vertexList[j + joinPlanes.size() * i].x;           polygon[1] = vertexList[j + joinPlanes.size() * i].y;           polygon[2] = vertexList[j + joinPlanes.size() * i].z;
                polygon[3] = vertexList[1 + j + joinPlanes.size() * i].x;       polygon[4] = vertexList[1 + j + joinPlanes.size() * i].y;       polygon[5] = vertexList[1 + j + joinPlanes.size() * i].z;
                polygon[6] = vertexList[1 + j + joinPlanes.size() * (i + 1)].x; polygon[7] = vertexList[1 + j + joinPlanes.size() * (i + 1)].y; polygon[8] = vertexList[1 + j + joinPlanes.size() * (i + 1)].z;
                polygon[9] = vertexList[j + joinPlanes.size() * (i + 1)].x;     polygon[10] = vertexList[j + joinPlanes.size() * (i + 1)].y;    polygon[11] = vertexList[j + joinPlanes.size() * (i + 1)].z;
            }
            else
            {
                //face->vertices.push_back(vertexList[j + joinPlanes.size() * i]);
                //face->vertices.push_back(vertexList[1 + j + joinPlanes.size() * i]);
                //face->vertices.push_back(vertexList[1 + j]);
                //face->vertices.push_back(vertexList[j]);
                polygon[0] = vertexList[j + joinPlanes.size() * i].x;     polygon[1] = vertexList[j + joinPlanes.size() * i].y;     polygon[2] = vertexList[j + joinPlanes.size() * i].z;
                polygon[3] = vertexList[1 + j + joinPlanes.size() * i].x; polygon[4] = vertexList[1 + j + joinPlanes.size() * i].y; polygon[5] = vertexList[1 + j + joinPlanes.size() * i].z;
                polygon[6] = vertexList[1 + j].x;                         polygon[7] = vertexList[1 + j].y;                         polygon[8] = vertexList[1 + j].z;
                polygon[9] = vertexList[j].x;                             polygon[10] = vertexList[j].y;                            polygon[11] = vertexList[j].z;
            }
            this->AddPolygon(polygon, 4);
        }
    }

    // end caps
    for (i = 0; i < rotatedProfile.size(); i++)
    {
        //face->vertices.push_back(vertexList[joinPlanes.size() * i]);
        polygon[i * 3] = vertexList[joinPlanes.size() * i].x;
        polygon[i * 3 + 1] = vertexList[joinPlanes.size() * i].y;
        polygon[i * 3 + 2] = vertexList[joinPlanes.size() * i].z;
    }
    this->AddPolygon(polygon, rotatedProfile.size());
    for (i = 0; i < rotatedProfile.size(); i++)
    {
        // face->vertices.push_back(vertexList[joinPlanes.size() - 1 + joinPlanes.size() * (rotatedProfile.size() - i - 1)]);
        polygon[i * 3] = vertexList[joinPlanes.size() - 1 + joinPlanes.size() * (rotatedProfile.size() - i - 1)].x;
        polygon[i * 3 + 1] = vertexList[joinPlanes.size() - 1 + joinPlanes.size() * (rotatedProfile.size() - i - 1)].y;
        polygon[i * 3 + 2] = vertexList[joinPlanes.size() - 1 + joinPlanes.size() * (rotatedProfile.size() - i - 1)].z;
    }
    this->AddPolygon(polygon, rotatedProfile.size());

    delete [] polygon;
}

// find intersection of line and plane
// returns true on success, false if no intersection
bool FacetedPolyline::Intersection(Line3D *line, Plane3D *plane, pgd::Vector *intersection)
{
    double denominator = line->direction * plane->GetNormal();
    double epsilon = 0.000001;
    double t;

    if (fabs(denominator) < epsilon)
    {
        // line and plane very close to parallel so they probably don't meet
        // but perhaps the origin is in the plane
        if (fabs(line->origin.x * plane->a + line->origin.y * plane->b + line->origin.z * plane->c + plane->d) > epsilon)
        {
            t = 0;
            *intersection = line->origin;
            return true;
        }
        else
        {
            return false;
        }
    }

    // compute intersection

    t = -(plane->a * line->origin.x + plane->b * line->origin.y + plane->c * line->origin.z + plane->d);
    t = t / denominator;
    *intersection = line->origin + t * line->direction;

    return true;

}

#endif

