/*
 *  FacetedObject.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <cfloat>
#include <vector>
#include <list>
#include <string>
#include <sstream>

#if defined(USE_QT)
#include <QFileInfo>
#include <QDir>
#include "GLUtils.h"
#endif

#include "FacetedObject.h"
#include "FacetedSphere.h"
#include "Face.h"
#include "DataFile.h"
#include "DebugControl.h"
#include "Util.h"
#include "PGDMath.h"


// create object
FacetedObject::FacetedObject()
{
    mNumVertices = 0;
    mNumVerticesAllocated = 0;
    mVertexList = 0;
    mNormalList = 0;
    mIndexList = 0;

    memset(m_DisplayPosition, 0, sizeof(dVector3));
    dRSetIdentity(m_DisplayRotation);

    m_UseRelativeOBJ = false;
    m_VerticesAsSpheresRadius = 0;

    m_AllocationIncrement = 8192;
    m_BadMesh = false;

    m_POVRayFile = 0;
    m_OBJFile = 0;

    m_vertexOffset = 0;

#ifdef USE_QT
    m_wireFrame = false;
    m_boundingBox = false;
    m_boundingBoxBuffers = false;
    m_normals = false;
    m_halfTransparency = false;
    m_meshBuffer = 0;
    m_mesh = 0;
    m_animatedMesh = 0;
    m_sceneNode = 0;
    m_simulationWindow = 0;
#endif
}

// destroy object
FacetedObject::~FacetedObject()
{
    if (mVertexList) delete mVertexList;
    if (mNormalList) delete mNormalList;
    if (mIndexList) delete mIndexList;
#ifdef USE_QT
    if (m_meshBuffer && m_meshBuffer->getReferenceCount() > 1)
        m_simulationWindow->videoDriver()->removeHardwareBuffer(m_meshBuffer); // this is really important because otherwise the driver keeps copies of the meshes
    if (m_sceneNode) m_sceneNode->remove();
#endif
}

// parse an OBJ file to a FacetedObject
// returns true on error
bool FacetedObject::ParseOBJFile(const char *filename, const pgd::Vector &scale, const pgd::Vector &offset)
{
    SetName(filename);
#ifdef USE_QT
    if (m_sceneNode) m_sceneNode->remove();
    // m_simulationWindow->sceneManager()->getParameters()->setAttribute(irr::scene::OBJ_LOADER_IGNORE_GROUPS, true); // not currently wanted
    irr::scene::IAnimatedMesh *mesh = m_simulationWindow->sceneManager()->getMesh(filename); // this works with several 3D asset formats
    if (mesh == 0) return true;


    irr::scene::IMeshManipulator *mm = m_simulationWindow->sceneManager()->getMeshManipulator();
#ifdef OBJ_LOADER_MIRRORS_X // the edited OBJ loader does not mirror X but the default irrlicht one does
    irr::core::vector3df objScale(-scale.x, scale.y, scale.z);
    mm->apply(irr::scene::SVertexPositionScaleManipulator(objScale), mesh, true); // this version sets most of the bounding boxes
#else
    if (scale.x != 1 && scale.y != 1 && scale.z != 1)
    {
        irr::core::vector3df objScale(scale.x, scale.y, scale.z);
        mm->apply(irr::scene::SVertexPositionScaleManipulator(objScale), mesh, true); // this version sets most of the bounding boxes
    }
#endif
    mm->flipSurfaces(mesh); // but I still need to reverse the winding for my meshes
    mm->recalculateNormals(mesh); // this might not be needed if the file contains normals

    // and do the offset as well
    irr::core::matrix4 m;
    irr::core::vector3df objOffset(offset.x, offset.y, offset.z);
    m.setTranslation(objOffset);
    mm->apply(irr::scene::SVertexPositionTransformManipulator(m), mesh, true); // this version sets most of the bounding boxes

    // IMeshManipulator does not set the bounding boxes for the animated mesh frames
    irr::core::aabbox3df box = mesh->getBoundingBox(); // mesh bounding box
    for (irr::u32 iFrame = 0; iFrame < mesh->getFrameCount(); iFrame++)
    {
        irr::scene::IMesh *frameMesh = mesh->getMesh(iFrame);
        irr::scene::SMesh *frameSMesh = dynamic_cast<irr::scene::SMesh *>(frameMesh);
        if (frameSMesh) { frameSMesh->recalculateBoundingBox(); continue; } //might need more cases depending on what sort of mesh we have
        mesh->getMesh(iFrame)->setBoundingBox(box); // this is a kludge because this bounding box isn't necessarily correct
    }


    irr::s32 idBitMask = 1 << 0;
    m_sceneNode = m_simulationWindow->sceneManager()->addAnimatedMeshSceneNode(mesh, 0, idBitMask);
    int minimalPolysPerNode = 256;
    irr::scene::ITriangleSelector* selector = m_simulationWindow->sceneManager()->createOctreeTriangleSelector(mesh, m_sceneNode, minimalPolysPerNode);
    m_sceneNode->setTriangleSelector(selector);
    selector->drop();
    irr::core::matrix4 matrix;
    matrix[0]=m_DisplayRotation[0];   matrix[1]=m_DisplayRotation[4];  matrix[2]=m_DisplayRotation[8];    matrix[3]=0;
    matrix[4]=m_DisplayRotation[1];   matrix[5]=m_DisplayRotation[5];  matrix[6]=m_DisplayRotation[9];    matrix[7]=0;
    matrix[8]=m_DisplayRotation[2];   matrix[9]=m_DisplayRotation[6];  matrix[10]=m_DisplayRotation[10];  matrix[11]=0;
    matrix[12]=m_DisplayPosition[0];  matrix[13]=m_DisplayPosition[1]; matrix[14]=m_DisplayPosition[2];   matrix[15]=1;
    m_sceneNode->setRotation(matrix.getRotationDegrees());
    m_sceneNode->setPosition(matrix.getTranslation());

    irr::s32 state = irr::scene::EDS_OFF;
    if (m_boundingBox) state |= irr::scene::EDS_BBOX;
    if (m_boundingBoxBuffers) state |= irr::scene::EDS_BBOX_BUFFERS;
    if (m_wireFrame) state |= irr::scene::EDS_MESH_WIRE_OVERLAY;
    if (m_normals) state |= irr::scene::EDS_NORMALS;
    if (m_halfTransparency) state |= irr::scene::EDS_HALF_TRANSPARENCY;
    m_sceneNode->setDebugDataVisible(state);
    m_sceneNode->setName(QString::asprintf("BODY GraphicsFile=\"%s\" Offset=\"%.17e %.17e %.17e\" Scale=\"%.17e %.17e %.17e\"",
                                            filename, offset.x, offset.y, offset.z, scale.x, scale.y, scale.z).toUtf8());
    m_sceneNode->setAutomaticCulling(irr::scene::EAC_OFF); // no object based culling wanted
    m_sceneNode->setMaterialFlag(irr::video::EMF_LIGHTING, true); // enable dynamic lighting
    m_sceneNode->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_sceneNode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);

    // add shadow
//    bool zfailmethod = true; // zfailmethod is better than zpass if camera is within the volume
//    irr::f32 infinity = 10.0f; // this value needs to be less than the camera far value for zfailmethod
//    m_sceneNode->addShadowVolumeSceneNode(0, -1, zfailmethod, infinity);

    return false;
#else
    DataFile theFile;
    theFile.ReadFile(filename);
    long bufferSizeLimit = theFile.GetSize() / 2; // if the file has any content it will have fewer lines than this
    char **linePtrs = new char *[bufferSizeLimit];
    long lineCount = DataFile::ReturnLines(theFile.GetRawData(), linePtrs, bufferSizeLimit);

    const int kBufferSize = 64000;
    char **tokens = new char *[kBufferSize];
    int numTokens;
    std::vector<pgd::Vector> vertexList;
    std::vector<std::vector<long> > faceList;
    std::vector<long> triFace(3);
    pgd::Vector vertex;
    double tri[9];
    long i, j;
    pgd::Vector min(DBL_MAX, DBL_MAX, DBL_MAX);
    pgd::Vector max(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    long spaceToAllocate = 0;
    int biggestPolygon = 0;
    double v;

    // parse the lines
    for  (i = 0; i < lineCount; i++)
    {
        // vertices
        if (linePtrs[i][0] == 'v' && linePtrs[i][1] == ' ')
        {
            numTokens = DataFile::ReturnTokens(linePtrs[i], tokens, kBufferSize);
            if (numTokens > 3)
            {
                vertex.x = atof(tokens[1]) * scale.x + offset.x;
                vertex.y = atof(tokens[2]) * scale.x + offset.x;
                vertex.z = atof(tokens[3]) * scale.x + offset.x;
                vertexList.push_back(vertex);

                if (gDebug == FacetedObjectDebug)
                {
                    min.x = MIN(min.x, vertex.x);
                    min.y = MIN(min.y, vertex.y);
                    min.z = MIN(min.z, vertex.z);
                    max.x = MAX(max.x, vertex.x);
                    max.y = MAX(max.y, vertex.y);
                    max.z = MAX(max.z, vertex.z);
                }
            }
        }

        // faces
        if (linePtrs[i][0] == 'f' && linePtrs[i][1] == ' ')
        {
            numTokens = DataFile::ReturnTokens(linePtrs[i], tokens, kBufferSize);
            if (numTokens == 4) // optimisation for triangles
            {
                triFace[0] = atol(tokens[1]) - 1;
                triFace[1] = atol(tokens[2]) - 1;
                triFace[2] = atol(tokens[3]) - 1;
                faceList.push_back(triFace);
                spaceToAllocate += 3;
                if (biggestPolygon < 3) biggestPolygon = 3;
                if (m_BadMesh) // currently duplicate the polygon but with reversed winding but this could be improved
                {
                    v = triFace[1];
                    triFace[1] = triFace[2];
                    triFace[2] = v;
                    faceList.push_back(triFace);
                    spaceToAllocate += 3;
                }

            }
            else
            {
                if (numTokens > 4)
                {
                    faceList.push_back(std::vector<long>(numTokens - 1));
                    // note obj files start at 1 not zero
                    for (j = 1; j < numTokens; j++)
                        faceList.back()[j - 1] = atol(tokens[j]) - 1;
                    spaceToAllocate += numTokens - 1;
                    if (biggestPolygon < numTokens - 1) biggestPolygon = numTokens - 1;
                    if (m_BadMesh) // currently duplicate the polygon but with reversed winding but this could be improved
                    {
                        faceList.push_back(std::vector<long>(numTokens - 1));
                        // note obj files start at 1 not zero
                        for (j = 1; j < numTokens; j++)
                            faceList.back()[j - 1] = atol(tokens[numTokens - j]) - 1;
                        spaceToAllocate += numTokens - 1;
                    }
                }
            }
        }
    }

    if (gDebug == FacetedObjectDebug)
        std::cerr << "ParseOBJFile:\tmin.x\t" << min.x << "\tmax.x\t" << max.x <<
                "\tmin.y\t" << min.y << "\tmax.y\t" << max.y <<
                "\tmin.z\t" << min.z << "\tmax.z\t" << max.z << "\n";


    if (m_VerticesAsSpheresRadius <= 0)
    {

        // fill out the display object
        m_AllocationIncrement = spaceToAllocate;
        if (biggestPolygon == 3) // optimise for triangular mesh
        {
            for (i = 0; i < (long)faceList.size(); i++)
            {
                for (j = 0; j < 3; j++)
                {
                    tri[j * 3] = vertexList[faceList[i][j]].x;
                    tri[j * 3 + 1] = vertexList[faceList[i][j]].y;
                    tri[j * 3 + 2] = vertexList[faceList[i][j]].z;
                }
                AddTriangle(tri);
            }
        }
        else
        {
            double *face = new double[biggestPolygon * 3];
            for (i = 0; i < (long)faceList.size(); i++)
            {
                if (faceList[i].size() == 3) // optimise for triangles
                {
                    for (j = 0; j < 3; j++)
                    {
                        tri[j * 3] = vertexList[faceList[i][j]].x;
                        tri[j * 3 + 1] = vertexList[faceList[i][j]].y;
                        tri[j * 3 + 2] = vertexList[faceList[i][j]].z;
                    }
                    AddTriangle(tri);
                }
                else
                {
                    for (j = 0; j < (long)faceList[i].size(); j++)
                    {
                        face[j * 3] = vertexList[faceList[i][j]].x;
                        face[j * 3 + 1] = vertexList[faceList[i][j]].y;
                        face[j * 3 + 2] = vertexList[faceList[i][j]].z;
                    }
                    AddPolygon(face, faceList[i].size());
                }
            }
            delete [] face;
        }
    }
    else
    {
        FacetedSphere masterSphere(m_VerticesAsSpheresRadius, 2);
        int nTri = masterSphere.GetNumTriangles();
        double *tri;
        pgd::Vector lastPos(0, 0, 0);
        m_AllocationIncrement = nTri * (int)vertexList.size();
        for (i = 0; i < (int)vertexList.size(); i++)
        {
            masterSphere.Move(vertexList[i].x - lastPos.x, vertexList[i].y - lastPos.y, vertexList[i].z - lastPos.z);
            lastPos = vertexList[i];
            for (j = 0; j < nTri; j++)
            {
                tri = masterSphere.GetTriangle(j);
                AddTriangle(tri);
            }
        }
    }

    // clear memory
    delete [] tokens;
    delete [] linePtrs;

    return false;
#endif
}

// write the object out as a POVRay string
// currently assumes all faces are triangles (call Triangulate if conversion is necessary)
void FacetedObject::WritePOVRay(std::ostringstream &theString)
{
    int i, j;
    double *vPtr;
    dVector3 prel, p, result;

    theString.precision(7); // should be plenty

    theString << "object {\n";
    theString << "  mesh {\n";

    // first faces
    for (i = 0; i < mNumVertices / 3; i++)
    {
        theString << "    triangle {\n";
        for (j = 0; j < 3; j++)
        {
            vPtr = mVertexList + i * 9 + j * 3;
            prel[0] = *vPtr++;
            prel[1] = *vPtr++;
            prel[2] = *vPtr;
            prel[3] = 0;
            dMULTIPLY0_331(p, m_DisplayRotation, prel);
            result[0] = p[0] + m_DisplayPosition[0];
            result[1] = p[1] + m_DisplayPosition[1];
            result[2] = p[2] + m_DisplayPosition[2];

            theString << "      <" << result[0] << "," << result[1] << "," << result[2] << ">\n";
        }
        theString << "    }\n";
    }

#ifdef USE_QT // this is only here because I don't define colours in the command line version - I might want to fix this
    // now colour
    theString << "    pigment {\n";
    theString << "      color rgbf<" << m_Colour.r << "," << m_Colour.g << "," << m_Colour.b <<"," << 1 - m_Colour.alpha << ">\n";
    theString << "    }\n";
    theString << "  }\n";
    theString << "}\n\n";
#endif
}

// Write a FacetedObject out as a OBJ
void FacetedObject::WriteOBJFile(std::ostringstream &out)
{
    int i, j;
    double *vPtr;
    dVector3 prel, p, result;
    static unsigned long counter = 0;

    out.precision(7); // should be plenty

    for (i = 0; i < (int)m_OBJName.size(); i++)
        if (m_OBJName[i] <= ' ') m_OBJName[i] = '_';
    out << "o " << m_OBJName << counter << "\n";
    counter++;

    if (m_UseRelativeOBJ)
    {
        // write out the vertices, faces, groups and objects
        // this is the relative version - inefficient but allows concatenation of objects
        for (i = 0; i < mNumVertices / 3; i++)
        {
            for (j = 0; j < 3; j++)
            {
                vPtr = mVertexList + i * 9 + j * 3;
                prel[0] = *vPtr++;
                prel[1] = *vPtr++;
                prel[2] = *vPtr;
                prel[3] = 0;
                dMULTIPLY0_331(p, m_DisplayRotation, prel);
                result[0] = p[0] + m_DisplayPosition[0];
                result[1] = p[1] + m_DisplayPosition[1];
                result[2] = p[2] + m_DisplayPosition[2];
                out << "v " << result[0] << " " << result[1] << " " << result[2] << "\n";
            }

            out << "f ";
            for (j = 0; j < 3; j++)
            {
                if (j == 3)
                    out << j - 3 << "\n";
                else
                    out << j - 3 << " ";
            }
        }
    }
    else
    {
        for (i = 0; i < mNumVertices / 3; i++)
        {
            for (j = 0; j < 3; j++)
            {
                vPtr = mVertexList + i * 9 + j * 3;
                prel[0] = *vPtr++;
                prel[1] = *vPtr++;
                prel[2] = *vPtr;
                prel[3] = 0;
                dMULTIPLY0_331(p, m_DisplayRotation, prel);
                result[0] = p[0] + m_DisplayPosition[0];
                result[1] = p[1] + m_DisplayPosition[1];
                result[2] = p[2] + m_DisplayPosition[2];
                out << "v " << result[0] << " " << result[1] << " " << result[2] << "\n";
            }
        }

        for (i = 0; i < mNumVertices / 3; i++)
        {
            out << "f ";
            for (j = 0; j < 3; j++)
            {
                // note this files vertex list start at 1 not zero
                if (j == 2)
                    out << *(mIndexList + i * 3 + j) + 1 + m_vertexOffset << "\n";
                else
                    out << *(mIndexList + i * 3 + j) + 1 + m_vertexOffset << " ";
            }
        }
        m_vertexOffset += mNumVertices;
    }
}

void FacetedObject::SetDisplayPosition(double x, double y, double z)
{
    m_DisplayPosition[0] = x;
    m_DisplayPosition[1] = y;
    m_DisplayPosition[2] = z;
}

void FacetedObject::SetDisplayRotation(const dMatrix3 R, bool fast)
{
    if (fast)
    {
        memcpy(m_DisplayRotation, R, sizeof(dMatrix3));
    }
    else
    {
        dQuaternion q;
        dRtoQ (R, q);
        dNormalize4 (q);
        dQtoR (q, m_DisplayRotation);
    }
}

// dQuaternion q [ w, x, y, z ], where w is the real part and (x, y, z) form the vector part.
void FacetedObject::SetDisplayRotationFromQuaternion(const dQuaternion q, bool fast)
{
    if (fast == false)
    {
        dQuaternion qq;
        memcpy(qq, q, sizeof(dQuaternion));
        dNormalize4 (qq);
        dQtoR(qq, m_DisplayRotation);
    }
    else
        dQtoR(q, m_DisplayRotation);
}

// this routine rotates the Z axis to point in a specified direction
void FacetedObject::SetDisplayRotationFromAxis(double x, double y, double z, bool fast)
{
    // calculate the rotation needed to get the axis pointing the right way
    dVector3 axis;
    axis[0] = x;
    axis[1] = y;
    axis[2] = z;
    if (fast == false) dNormalize3(axis);
    dVector3 p, q;
    // calculate 2 perpendicular vectors
    dPlaneSpace(axis, p, q);
    // assemble the matrix
    m_DisplayRotation[3] = m_DisplayRotation[7] = m_DisplayRotation[11] = 0;

    m_DisplayRotation[0] =    p[0]; m_DisplayRotation[4] =    p[1]; m_DisplayRotation[8] =     p[2];
    m_DisplayRotation[1] =    q[0]; m_DisplayRotation[5] =    q[1]; m_DisplayRotation[9] =     q[2];
    m_DisplayRotation[2] = axis[0]; m_DisplayRotation[6] = axis[1]; m_DisplayRotation[10] = axis[2];
}

// utility to calculate a face normal
// this assumes anticlockwise winding
void FacetedObject::ComputeFaceNormal(const double *v1, const double *v2, const double *v3, double normal[3])
{
    double a[3], b[3];

    // calculate in plane vectors
    a[0] = v2[0] - v1[0];
    a[1] = v2[1] - v1[1];
    a[2] = v2[2] - v1[2];
    b[0] = v3[0] - v1[0];
    b[1] = v3[1] - v1[1];
    b[2] = v3[2] - v1[2];

    // cross(a, b, normal);
    normal[0] = a[1] * b[2] - a[2] * b[1];
    normal[1] = a[2] * b[0] - a[0] * b[2];
    normal[2] = a[0] * b[1] - a[1] * b[0];

    // normalize(normal);
    double norm = sqrt(normal[0] * normal[0] +
                      normal[1] * normal[1] +
                      normal[2] * normal[2]);

    if (norm > 0.0)
    {
        normal[0] /= norm;
        normal[1] /= norm;
        normal[2] /= norm;
    }
}

// move the object
// note this must be used before first draw call
void FacetedObject::Move(double x, double y, double z)
{
    for (int i = 0; i < mNumVertices; i++)
    {
        mVertexList[i * 3] += x;
        mVertexList[i * 3 + 1] += y;
        mVertexList[i * 3 + 2] += z;
    }
#ifdef USE_QT
    if (m_sceneNode)
    {
        irr::scene::IMesh *mesh = m_sceneNode->getMesh();
        if (mesh)
        {
            irr::scene::IMeshManipulator *mm = m_simulationWindow->sceneManager()->getMeshManipulator();
            irr::core::matrix4 m;
            irr::core::vector3df objOffset(x, y, z);
            m.setTranslation(objOffset);
            mm->apply(irr::scene::SVertexPositionTransformManipulator(m), mesh, true); // this version sets most of the bounding boxes

            irr::scene::IAnimatedMesh *animatedMesh = dynamic_cast<irr::scene::IAnimatedMesh *>(mesh);
            if (animatedMesh)
            {
                // IMeshManipulator does not set the bounding boxes for the animated mesh frames
                irr::core::aabbox3df box = mesh->getBoundingBox(); // mesh bounding box
                for (irr::u32 iFrame = 0; iFrame < animatedMesh->getFrameCount(); iFrame++)
                {
                    irr::scene::IMesh *frameMesh = animatedMesh->getMesh(iFrame);
                    irr::scene::SMesh *frameSMesh = dynamic_cast<irr::scene::SMesh *>(frameMesh);
                    if (frameSMesh) { frameSMesh->recalculateBoundingBox(); continue; } //might need more cases depending on what sort of mesh we have
                    animatedMesh->getMesh(iFrame)->setBoundingBox(box); // this is a kludge because this bounding box isn't necessarily correct
                }
            }
        }
    }
#endif

}

// scale the object
// note this must be used before first draw call
void FacetedObject::Scale(double x, double y, double z)
{
    for (int i = 0; i < mNumVertices; i++)
    {
        mVertexList[i * 3] *= x;
        mVertexList[i * 3 + 1] *= y;
        mVertexList[i * 3 + 2] *= z;
    }
#ifdef USE_QT
    if (m_sceneNode)
    {
        irr::scene::IMesh *mesh = m_sceneNode->getMesh();
        if (mesh)
        {
            irr::scene::IMeshManipulator *mm = m_simulationWindow->sceneManager()->getMeshManipulator();
            irr::core::vector3df objScale(x, y, z);
            mm->apply(irr::scene::SVertexPositionScaleManipulator(objScale), mesh, true); // this version sets most of the bounding boxes
            mm->recalculateNormals(mesh); // this might not be needed but it is safer

            irr::scene::IAnimatedMesh *animatedMesh = dynamic_cast<irr::scene::IAnimatedMesh *>(mesh);
            if (animatedMesh)
            {
                // IMeshManipulator does not set the bounding boxes for the animated mesh frames
                irr::core::aabbox3df box = mesh->getBoundingBox(); // mesh bounding box
                for (irr::u32 iFrame = 0; iFrame < animatedMesh->getFrameCount(); iFrame++)
                {
                    irr::scene::IMesh *frameMesh = animatedMesh->getMesh(iFrame);
                    irr::scene::SMesh *frameSMesh = dynamic_cast<irr::scene::SMesh *>(frameMesh);
                    if (frameSMesh) { frameSMesh->recalculateBoundingBox(); continue; } //might need more cases depending on what sort of mesh we have
                    animatedMesh->getMesh(iFrame)->setBoundingBox(box); // this is a kludge because this bounding box isn't necessarily correct
                }
            }
        }
    }
#endif
}

// this routine triangulates the polygon and calls AddTriangle to do the actual data adding
// vertices are a packed list of floating point numbers
// x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4
void FacetedObject::AddPolygon(const double *vertices, int nSides)
{
    // add faces as triangles
    double triangle[9];
    triangle[0] = vertices[0];
    triangle[1] = vertices[1];
    triangle[2] = vertices[2];
    for (int j = 2; j < nSides; j++)
    {
        triangle[3] = vertices[(j - 1) * 3];
        triangle[4] = vertices[(j - 1) * 3 + 1];
        triangle[5] = vertices[(j - 1) * 3 + 2];
        triangle[6] = vertices[(j * 3)];
        triangle[7] = vertices[(j * 3) + 1];
        triangle[8] = vertices[(j * 3) + 2];
        AddTriangle(triangle);
    }
}


// this is the only routine that actually adds data to the facetted object
// it gets called by add polygon
// vertices is a packed list of floating point numbers
// x1, y1, z1, x2, y2, z2, x3, y3, z3
void FacetedObject::AddTriangle(const double *vertices)
{
    int newNumVertices = mNumVertices + 3;
    if (newNumVertices > mNumVerticesAllocated)
    {
        AllocateMemory(mNumVerticesAllocated + m_AllocationIncrement);
        m_AllocationIncrement *= 2;
    }
    memcpy(mVertexList + mNumVertices * 3, vertices, sizeof(double) * 9);

    // now calculate the normals
    double normal[3];
    ComputeFaceNormal(vertices, vertices + 3, vertices + 6, normal);
    memcpy(mNormalList + mNumVertices * 3, normal, sizeof(double) * 3);
    memcpy(mNormalList + mNumVertices * 3 + 3, normal, sizeof(double) * 3);
    memcpy(mNormalList + mNumVertices * 3 + 6, normal, sizeof(double) * 3);

    // and finally the indices
    mIndexList[mNumVertices] = mNumVertices;
    mNumVertices++;
    mIndexList[mNumVertices] = mNumVertices;
    mNumVertices++;
    mIndexList[mNumVertices] = mNumVertices;
    mNumVertices++;
}

// this routine handles the memory allocation
void FacetedObject::AllocateMemory(int allocation)
{
    if (allocation > mNumVerticesAllocated)
    {
        mNumVerticesAllocated = allocation;
        double *newVertexList = new double[mNumVerticesAllocated * 3];
        double *newNormalList = new double[mNumVerticesAllocated * 3];
        int *newIndexList = new int[mNumVerticesAllocated];
        if (mVertexList)
        {
            memcpy(newVertexList, mVertexList, sizeof(double) * mNumVertices * 3);
            delete [] mVertexList;
            memcpy(newNormalList, mNormalList, sizeof(double) * mNumVertices * 3);
            delete [] mNormalList;
            memcpy(newIndexList, mIndexList, sizeof(int) * mNumVertices);
            delete [] mIndexList;
        }
        mVertexList = newVertexList;
        mNormalList = newNormalList;
        mIndexList = newIndexList;
    }
}

// return an ODE style trimesh
// note memory is allocated by this routine and will need to be released elsewhere
void FacetedObject::CalculateTrimesh(double **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride)
{
    int i;
    *vertexStride = 3 * sizeof(double);
    *triStride = 3 * sizeof(dTriIndex);

    *numVertices = mNumVertices;
    *numTriIndexes = mNumVertices;

    *vertices = new double[mNumVertices * 3];
    *triIndexes = new dTriIndex[mNumVertices];

    for (i = 0; i < mNumVertices; i++)
    {
        (*vertices)[i * 3] = mVertexList[i * 3];
        (*vertices)[i * 3 + 1] = mVertexList[i * 3 + 1];
        (*vertices)[i * 3 + 2] = mVertexList[i * 3 + 2];
    }

    for (i = 0; i < mNumVertices; i++)
    {
        (*triIndexes)[i] = mIndexList[i];
    }
}

// return an ODE style trimesh
// note memory is allocated by this routine and will need to be released elsewhere
void FacetedObject::CalculateTrimesh(float **vertices, int *numVertices, int *vertexStride, dTriIndex **triIndexes, int *numTriIndexes, int *triStride)
{
    int i;
    *vertexStride = 3 * sizeof(float);
    *triStride = 3 * sizeof(dTriIndex);

    *numVertices = mNumVertices;
    *numTriIndexes = mNumVertices;

    *vertices = new float[mNumVertices * 3];
    *triIndexes = new dTriIndex[mNumVertices];

    for (i = 0; i < mNumVertices; i++)
    {
        (*vertices)[i * 3] = mVertexList[i * 3];
        (*vertices)[i * 3 + 1] = mVertexList[i * 3 + 1];
        (*vertices)[i * 3 + 2] = mVertexList[i * 3 + 2];
    }

    for (i = 0; i < mNumVertices; i++)
    {
        (*triIndexes)[i] = mIndexList[i];
    }
}

// calculate mass properties
// based on dMassSetTrimesh
/*
 * dMassSetTrimesh, implementation by Gero Mueller.
 * Based on Brian Mirtich, "Fast and Accurate Computation of
 * Polyhedral Mass Properties," journal of graphics tools, volume 1,
 * number 2, 1996.
 */

#define	SQR(x)			((x)*(x))						//!< Returns x square
#define	CUBE(x)			((x)*(x)*(x))					//!< Returns x cube

#define _I(i,j) I[(i)*4+(j)]

void FacetedObject::CalculateMassProperties(dMass *m, double density, bool clockwise)
{
    dMassSetZero (m);

#ifdef USE_QT

    // assumes anticlockwise winding

    double nx, ny, nz;
    unsigned int j, A, B, C;
    // face integrals
    double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

    // projection integrals
    double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

    double T0 = 0;
    double T1[3] = {0., 0., 0.};
    double T2[3] = {0., 0., 0.};
    double TP[3] = {0., 0., 0.};

    dVector3 v[3];

    irr::scene::IMesh *frameMesh = m_sceneNode->getMesh()->getMesh(0); // get the first animated mesh frame (there should only be one anyway)
    irr::u32 meshBufferCount = frameMesh->getMeshBufferCount();
    for (irr::u32 iMeshBuffer = 0; iMeshBuffer < meshBufferCount; iMeshBuffer++) // loop over the mesh buffers
    {
        irr::scene::IMeshBuffer *meshBuffer = frameMesh->getMeshBuffer(iMeshBuffer);
        irr::video::E_VERTEX_TYPE vertexType = meshBuffer->getVertexType();
        if (vertexType == irr::video::EVT_STANDARD) // need different pointers for each vertex type
        {
            irr::video::S3DVertex *vertices = (irr::video::S3DVertex *)meshBuffer->getVertices();
            irr::scene::IDynamicMeshBuffer *dynamicMeshBuffer = dynamic_cast<irr::scene::IDynamicMeshBuffer *>(meshBuffer);
            if (dynamicMeshBuffer)
            {
                irr::u32 indexCount = dynamicMeshBuffer->getIndexCount();
                if (dynamicMeshBuffer->getIndexType() == irr::video::EIT_32BIT)
                {
                    irr::u32 *indices = (irr::u32 *)meshBuffer->getIndices();

                    for (irr::u32 index = 0; index < indexCount; index += 3)
                    {

                        if (clockwise == false)
                        {
                            for (j = 0; j < 3; j++)
                            {
                                v[j][0] = vertices[indices[index + j]].Pos.X; // mVertexList[i * 9 + j * 3];
                                v[j][1] = vertices[indices[index + j]].Pos.Y; // mVertexList[i * 9 + j * 3 + 1];
                                v[j][2] = vertices[indices[index + j]].Pos.Z; // mVertexList[i * 9 + j * 3 + 2];
                            }
                        }
                        else
                        {
                            for (j = 0; j < 3; j++)
                            {
                                v[j][2] = vertices[indices[index + j]].Pos.X; // mVertexList[i * 9 + j * 3];
                                v[j][1] = vertices[indices[index + j]].Pos.Y; // mVertexList[i * 9 + j * 3 + 1];
                                v[j][0] = vertices[indices[index + j]].Pos.Z; // mVertexList[i * 9 + j * 3 + 2];
                            }
                        }

                        dVector3 n, a, b;
                        dOP( a, -, v[1], v[0] );
                        dOP( b, -, v[2], v[0] );
                        dCROSS( n, =, b, a );
                        nx = fabs(n[0]);
                        ny = fabs(n[1]);
                        nz = fabs(n[2]);

                        if( nx > ny && nx > nz )
                            C = 0;
                        else
                            C = (ny > nz) ? 1 : 2;

                        // Even though all triangles might be initially valid,
                        // a triangle may degenerate into a segment after applying
                        // space transformation.
                        if (n[C] != REAL(0.0))
                        {
                            A = (C + 1) % 3;
                            B = (A + 1) % 3;

                            // calculate face integrals
                            {
                                double w;
                                double k1, k2, k3, k4;

                                //compProjectionIntegrals(f);
                                {
                                    double a0, a1, da;
                                    double b0, b1, db;
                                    double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
                                    double a1_2, a1_3, b1_2, b1_3;
                                    double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
                                    double Cab, Kab, Caab, Kaab, Cabb, Kabb;

                                    P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

                                    for( j = 0; j < 3; j++)
                                    {
                                        switch(j)
                                        {
                                        case 0:
                                            a0 = v[0][A];
                                            b0 = v[0][B];
                                            a1 = v[1][A];
                                            b1 = v[1][B];
                                            break;
                                        case 1:
                                            a0 = v[1][A];
                                            b0 = v[1][B];
                                            a1 = v[2][A];
                                            b1 = v[2][B];
                                            break;
                                        case 2:
                                            a0 = v[2][A];
                                            b0 = v[2][B];
                                            a1 = v[0][A];
                                            b1 = v[0][B];
                                            break;
                                        }
                                        da = a1 - a0;
                                        db = b1 - b0;
                                        a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
                                        b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
                                        a1_2 = a1 * a1; a1_3 = a1_2 * a1;
                                        b1_2 = b1 * b1; b1_3 = b1_2 * b1;

                                        C1 = a1 + a0;
                                        Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
                                        Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
                                        Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
                                        Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
                                        Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
                                        Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

                                        P1 += db*C1;
                                        Pa += db*Ca;
                                        Paa += db*Caa;
                                        Paaa += db*Caaa;
                                        Pb += da*Cb;
                                        Pbb += da*Cbb;
                                        Pbbb += da*Cbbb;
                                        Pab += db*(b1*Cab + b0*Kab);
                                        Paab += db*(b1*Caab + b0*Kaab);
                                        Pabb += da*(a1*Cabb + a0*Kabb);
                                    }

                                    P1 /= 2.0;
                                    Pa /= 6.0;
                                    Paa /= 12.0;
                                    Paaa /= 20.0;
                                    Pb /= -6.0;
                                    Pbb /= -12.0;
                                    Pbbb /= -20.0;
                                    Pab /= 24.0;
                                    Paab /= 60.0;
                                    Pabb /= -60.0;
                                }

                                w = - dDOT(n, v[0]);

                                k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

                                Fa = k1 * Pa;
                                Fb = k1 * Pb;
                                Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

                                Faa = k1 * Paa;
                                Fbb = k1 * Pbb;
                                Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb +
                                            w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

                                Faaa = k1 * Paaa;
                                Fbbb = k1 * Pbbb;
                                Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab
                                              + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
                                              + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
                                              + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

                                Faab = k1 * Paab;
                                Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
                                Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
                                             + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
                            }


                            T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

                            T1[A] += n[A] * Faa;
                            T1[B] += n[B] * Fbb;
                            T1[C] += n[C] * Fcc;
                            T2[A] += n[A] * Faaa;
                            T2[B] += n[B] * Fbbb;
                            T2[C] += n[C] * Fccc;
                            TP[A] += n[A] * Faab;
                            TP[B] += n[B] * Fbbc;
                            TP[C] += n[C] * Fcca;
                        }
                    }
                }
            }
        }
    }

    T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
    T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
    TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;

    m->mass = density * T0;
    m->_I(0,0) = density * (T2[1] + T2[2]);
    m->_I(1,1) = density * (T2[2] + T2[0]);
    m->_I(2,2) = density * (T2[0] + T2[1]);
    m->_I(0,1) = - density * TP[0];
    m->_I(1,0) = - density * TP[0];
    m->_I(2,1) = - density * TP[1];
    m->_I(1,2) = - density * TP[1];
    m->_I(2,0) = - density * TP[2];
    m->_I(0,2) = - density * TP[2];

    m->c[0] = T1[0] / T0;
    m->c[1] = T1[1] / T0;
    m->c[2] = T1[2] / T0;
#else
    // assumes anticlockwise winding

    unsigned int triangles = mNumVertices / 3;

    double nx, ny, nz;
    unsigned int i, j, A, B, C;
    // face integrals
    double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

    // projection integrals
    double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

    double T0 = 0;
    double T1[3] = {0., 0., 0.};
    double T2[3] = {0., 0., 0.};
    double TP[3] = {0., 0., 0.};

    dVector3 v[3];
    for( i = 0; i < triangles; i++ )
    {
        if (clockwise == false)
        {
            for (j = 0; j < 3; j++)
            {
                v[j][0] = mVertexList[i * 9 + j * 3];
                v[j][1] = mVertexList[i * 9 + j * 3 + 1];
                v[j][2] = mVertexList[i * 9 + j * 3 + 2];
            }
        }
        else
        {
            for (j = 0; j < 3; j++)
            {
                v[j][2] = mVertexList[i * 9 + j * 3];
                v[j][1] = mVertexList[i * 9 + j * 3 + 1];
                v[j][0] = mVertexList[i * 9 + j * 3 + 2];
            }
        }

        dVector3 n, a, b;
        dOP( a, -, v[1], v[0] );
        dOP( b, -, v[2], v[0] );
        dCROSS( n, =, b, a );
        nx = fabs(n[0]);
        ny = fabs(n[1]);
        nz = fabs(n[2]);

        if( nx > ny && nx > nz )
            C = 0;
        else
            C = (ny > nz) ? 1 : 2;

        // Even though all triangles might be initially valid,
        // a triangle may degenerate into a segment after applying
        // space transformation.
        if (n[C] != REAL(0.0))
        {
            A = (C + 1) % 3;
            B = (A + 1) % 3;

            // calculate face integrals
            {
                double w;
                double k1, k2, k3, k4;

                //compProjectionIntegrals(f);
                {
                    double a0, a1, da;
                    double b0, b1, db;
                    double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
                    double a1_2, a1_3, b1_2, b1_3;
                    double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
                    double Cab, Kab, Caab, Kaab, Cabb, Kabb;

                    P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

                    for( j = 0; j < 3; j++)
                    {
                        switch(j)
                        {
                        case 0:
                            a0 = v[0][A];
                            b0 = v[0][B];
                            a1 = v[1][A];
                            b1 = v[1][B];
                            break;
                        case 1:
                            a0 = v[1][A];
                            b0 = v[1][B];
                            a1 = v[2][A];
                            b1 = v[2][B];
                            break;
                        case 2:
                            a0 = v[2][A];
                            b0 = v[2][B];
                            a1 = v[0][A];
                            b1 = v[0][B];
                            break;
                        }
                        da = a1 - a0;
                        db = b1 - b0;
                        a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
                        b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
                        a1_2 = a1 * a1; a1_3 = a1_2 * a1;
                        b1_2 = b1 * b1; b1_3 = b1_2 * b1;

                        C1 = a1 + a0;
                        Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
                        Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
                        Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
                        Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
                        Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
                        Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

                        P1 += db*C1;
                        Pa += db*Ca;
                        Paa += db*Caa;
                        Paaa += db*Caaa;
                        Pb += da*Cb;
                        Pbb += da*Cbb;
                        Pbbb += da*Cbbb;
                        Pab += db*(b1*Cab + b0*Kab);
                        Paab += db*(b1*Caab + b0*Kaab);
                        Pabb += da*(a1*Cabb + a0*Kabb);
                    }

                    P1 /= 2.0;
                    Pa /= 6.0;
                    Paa /= 12.0;
                    Paaa /= 20.0;
                    Pb /= -6.0;
                    Pbb /= -12.0;
                    Pbbb /= -20.0;
                    Pab /= 24.0;
                    Paab /= 60.0;
                    Pabb /= -60.0;
                }

                w = - dDOT(n, v[0]);

                k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

                Fa = k1 * Pa;
                Fb = k1 * Pb;
                Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

                Faa = k1 * Paa;
                Fbb = k1 * Pbb;
                Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb +
                            w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

                Faaa = k1 * Paaa;
                Fbbb = k1 * Pbbb;
                Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab
                              + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
                              + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
                              + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

                Faab = k1 * Paab;
                Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
                Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
                             + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
            }


            T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

            T1[A] += n[A] * Faa;
            T1[B] += n[B] * Fbb;
            T1[C] += n[C] * Fcc;
            T2[A] += n[A] * Faaa;
            T2[B] += n[B] * Fbbb;
            T2[C] += n[C] * Fccc;
            TP[A] += n[A] * Faab;
            TP[B] += n[B] * Fbbc;
            TP[C] += n[C] * Fcca;
        }
    }

    T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
    T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
    TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;

    m->mass = density * T0;
    m->_I(0,0) = density * (T2[1] + T2[2]);
    m->_I(1,1) = density * (T2[2] + T2[0]);
    m->_I(2,2) = density * (T2[0] + T2[1]);
    m->_I(0,1) = - density * TP[0];
    m->_I(1,0) = - density * TP[0];
    m->_I(2,1) = - density * TP[1];
    m->_I(1,2) = - density * TP[1];
    m->_I(2,0) = - density * TP[2];
    m->_I(0,2) = - density * TP[2];

    m->c[0] = T1[0] / T0;
    m->c[1] = T1[1] / T0;
    m->c[2] = T1[2] / T0;
#endif
}

// reverse the face winding
void FacetedObject::ReverseWinding()
{
    double t;
    int numTriangles = mNumVertices / 3;
    int i, j;
    for (i = 0; i < numTriangles; i++)
    {
        for (j = 0; j < 3; j++)
        {
            t = mVertexList[i * 9 + 3 + j];
            mVertexList[i * 9 + 3 + j] = mVertexList[i * 9 + 6 + j];
            mVertexList[i * 9 + 6 + j] = t;
        }
    }
    for (i = 0; i < mNumVertices * 3; i++) mNormalList[i] = -mNormalList[i];
}

// add the faces from one faceted object to another
void FacetedObject::AddFacetedObject(FacetedObject *object, bool useDisplayRotation)
{
    int numTriangles = object->GetNumTriangles();
    double *triangle, *p1;
    double triangle2[9];
    dVector3 v1, v1r;

    if (useDisplayRotation)
    {
        for (int i = 0; i < numTriangles; i++)
        {
            triangle = object->GetTriangle(i);
            for (int j =0; j < 3; j++)
            {
                p1 = triangle + 3 * j;
                v1[0] = p1[0];
                v1[1] = p1[1];
                v1[2] = p1[2];
                v1[3] = 0;
                dMULTIPLY0_331(v1r, m_DisplayRotation, v1);
                p1 = triangle2 + 3 * j;
                p1[0] = v1r[0] + m_DisplayPosition[0];
                p1[1] = v1r[1] + m_DisplayPosition[1];
                p1[2] = v1r[2] + m_DisplayPosition[2];
            }
            AddTriangle(triangle2);
        }
    }
    else
    {
        for (int i = 0; i < numTriangles; i++)
        {
            triangle = object->GetTriangle(i);
            AddTriangle(triangle);
        }
    }
}

void FacetedObject::Draw()
{
#ifdef USE_QT
    if (m_sceneNode == 0)
    {
        initializeSceneNode(); // this must be the first run through
        if (m_sceneNode == 0) return;
    }

    irr::core::matrix4 matrix;
    matrix[0]=m_DisplayRotation[0];   matrix[1]=m_DisplayRotation[4];  matrix[2]=m_DisplayRotation[8];    matrix[3]=0;
    matrix[4]=m_DisplayRotation[1];   matrix[5]=m_DisplayRotation[5];  matrix[6]=m_DisplayRotation[9];    matrix[7]=0;
    matrix[8]=m_DisplayRotation[2];   matrix[9]=m_DisplayRotation[6];  matrix[10]=m_DisplayRotation[10];  matrix[11]=0;
    matrix[12]=m_DisplayPosition[0];  matrix[13]=m_DisplayPosition[1]; matrix[14]=m_DisplayPosition[2];   matrix[15]=1;
    m_sceneNode->setRotation(matrix.getRotationDegrees());
    m_sceneNode->setPosition(matrix.getTranslation());
    m_sceneNode->setVisible(m_Visible);
#endif

//    if (m_destinationPOVRay && m_Visible)
//    {
//        std::ostringstream theString;
//        WritePOVRay(theString);
//        if (m_POVRayFile) (*m_POVRayFile) << theString.str();
//        else std::cout << theString.str();
//    }

//    if (m_destinationOBJFile && m_Visible)
//    {
//        std::ostringstream theString;
//        WriteOBJFile(theString);
//        if (m_OBJFile) (*m_OBJFile) << theString.str();
//        else std::cout << theString.str();
//    }
}

#ifdef USE_QT
void FacetedObject::initializeSceneNode()
{
    if (m_simulationWindow == 0) return;

    m_meshBuffer = new irr::scene::CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
    m_meshBuffer->getVertexBuffer().reallocate(GetNumVertices());
    m_meshBuffer->getIndexBuffer().reallocate(GetNumTriangles() * 3);
    m_meshBuffer->setHardwareMappingHint(irr::scene::EHM_STATIC);

    // qDebug("Colour=%f,%f,%f", colour->r, colour->g, colour->b);
    float r = GetColour()->r;
    float g = GetColour()->g;
    float b = GetColour()->b;
    float a = GetColour()->alpha;
    int i;
    irr::video::S3DVertex v;
    irr::video::SColor vertexColour(255 * a, 255 * r, 255 * g, 255 * b);
    irr::f32 minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX;
    irr::f32 maxx = -FLT_MAX, maxy = -FLT_MAX, maxz = -FLT_MAX;

    double *vertexList = GetVertexList();
    for (i = 0; i < GetNumVertices(); i++)
    {
        v.Pos.set(vertexList[0], vertexList[1], vertexList[2]);
        // v.Color.set(vertexColour.color); // this is now done with SetMeshColour
        // v.Normal.set(nx, ny, nz);
        // v.TCoords.set(tu, tv);
        m_meshBuffer->getVertexBuffer().push_back(v);
        if (v.Pos.X < minx) minx = v.Pos.X;
        if (v.Pos.Y < miny) miny = v.Pos.Y;
        if (v.Pos.Z < minz) minz = v.Pos.Z;
        if (v.Pos.X > maxx) maxx = v.Pos.X;
        if (v.Pos.Y > maxy) maxy = v.Pos.Y;
        if (v.Pos.Z > maxz) maxz = v.Pos.Z;
        vertexList += 3;
    }

    int *indexList = GetIndexList();
    for (i = 0; i < GetNumTriangles(); i++)
    {
        m_meshBuffer->getIndexBuffer().push_back(*indexList++);
        m_meshBuffer->getIndexBuffer().push_back(*indexList++);
        m_meshBuffer->getIndexBuffer().push_back(*indexList++);
    }

    m_meshBuffer->setBoundingBox(irr::core::aabbox3df(minx, miny, minz, maxx, maxy, maxz));
    m_simulationWindow->sceneManager()->getMeshManipulator()->recalculateNormals(m_meshBuffer);

    m_mesh = new irr::scene::SMesh();
    m_mesh->addMeshBuffer(m_meshBuffer);
    m_mesh->recalculateBoundingBox();
    m_simulationWindow->SetMeshColour(m_mesh, vertexColour);
    m_meshBuffer->drop();

    m_animatedMesh = new irr::scene::SAnimatedMesh();
    m_animatedMesh->addMesh(m_mesh);
    m_mesh->drop();

    irr::s32 idBitMask = 1 << 0;
    m_sceneNode = m_simulationWindow->sceneManager()->addAnimatedMeshSceneNode(m_animatedMesh, 0, idBitMask);
    m_animatedMesh->drop();
    int minimalPolysPerNode = 256;
    irr::scene::ITriangleSelector* selector = m_simulationWindow->sceneManager()->createOctreeTriangleSelector(m_sceneNode->getMesh(), m_sceneNode, minimalPolysPerNode);
    m_sceneNode->setTriangleSelector(selector);
    selector->drop();
    irr::core::matrix4 matrix;
    matrix[0]=m_DisplayRotation[0];   matrix[1]=m_DisplayRotation[4];  matrix[2]=m_DisplayRotation[8];    matrix[3]=0;
    matrix[4]=m_DisplayRotation[1];   matrix[5]=m_DisplayRotation[5];  matrix[6]=m_DisplayRotation[9];    matrix[7]=0;
    matrix[8]=m_DisplayRotation[2];   matrix[9]=m_DisplayRotation[6];  matrix[10]=m_DisplayRotation[10];  matrix[11]=0;
    matrix[12]=m_DisplayPosition[0];  matrix[13]=m_DisplayPosition[1]; matrix[14]=m_DisplayPosition[2];   matrix[15]=1;
    m_sceneNode->setRotation(matrix.getRotationDegrees());
    m_sceneNode->setPosition(matrix.getTranslation());

    irr::s32 state = irr::scene::EDS_OFF;
    if (m_boundingBox) state |= irr::scene::EDS_BBOX;
    if (m_boundingBoxBuffers) state |= irr::scene::EDS_BBOX_BUFFERS;
    if (m_wireFrame) state |= irr::scene::EDS_MESH_WIRE_OVERLAY;
    if (m_normals) state |= irr::scene::EDS_NORMALS;
    if (m_halfTransparency) state |= irr::scene::EDS_HALF_TRANSPARENCY;
    m_sceneNode->setDebugDataVisible(state);
    m_sceneNode->setName(GetName()->c_str());
    m_sceneNode->setAutomaticCulling(irr::scene::EAC_OFF); // no object based culling wanted
    m_sceneNode->setMaterialFlag(irr::video::EMF_LIGHTING, true); // enable dynamic lighting
}

SimulationWindow *FacetedObject::simulationWindow() const
{
    return m_simulationWindow;
}

void FacetedObject::setSimulationWindow(SimulationWindow *simulationWindow)
{
    m_simulationWindow = simulationWindow;
}

irr::scene::IAnimatedMeshSceneNode *FacetedObject::sceneNode() const
{
    return m_sceneNode;
}

void FacetedObject::setTexture(irr::u32 textureLayer, irr::video::ITexture *texture)
{
    if (m_sceneNode) m_sceneNode->setMaterialTexture(textureLayer, texture);
}

void FacetedObject::setMaterialFlag(irr::video::E_MATERIAL_FLAG flag, bool newvalue)
{
    if (m_sceneNode) m_sceneNode->setMaterialFlag(flag, newvalue);
}


#endif
