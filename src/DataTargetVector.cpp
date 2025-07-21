/*
 *  DataTargetVector.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include <ode/ode.h>

#include "DataTargetVector.h"
#include "Body.h"
#include "PGDMath.h"
#include "Util.h"
#include "DataFile.h"
#include "HingeJoint.h"
#include "BallJoint.h"
#include "UniversalJoint.h"
#include "PositionReporter.h"
#include "Geom.h"
#ifdef USE_QT
#include "FacetedSphere.h"
#endif

DataTargetVector::DataTargetVector()
{
    m_ValueList = 0;
    m_ValueListLength = -1;
#ifdef USE_QT
    mRadius = 0.005;
    mOnlyDrawNext = false;
    m_axisSceneNode = 0;
#endif
}

DataTargetVector::~DataTargetVector()
{
    if (m_TargetTimeList) delete [] m_ValueList;
}

// note in this case the pointer is to a list of the elements of
// size vectors
void DataTargetVector::SetTargetValues(int size, double *values)
{
    int i;
    if (size != m_TargetTimeListLength)
    {
        std::cerr << "DataTargetVector::SetTargetValues error: size = " << size << "\n";
        return;
    }
    if (m_ValueListLength != size)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = size;
        m_ValueList = new pgd::Vector[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        m_ValueList[i].x = values[i * 3];
        m_ValueList[i].y = values[i * 3 + 1];
        m_ValueList[i].z = values[i * 3 + 2];
    }
}

// note in this case the pointer is to a string which is a list of the elements of
// size vector
void DataTargetVector::SetTargetValues(const char *buf)
{
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    int i;

    strcpy(lBuf, buf);
    int size = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (size != m_TargetTimeListLength * 3)
    {
        std::cerr << "DataTargetVector::SetTargetValues error: size = " << size << "\n";
        return;
    }

    if (m_ValueListLength != m_TargetTimeListLength)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = m_TargetTimeListLength;
        m_ValueList = new pgd::Vector[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        m_ValueList[i].x = Util::Double(lBufPtrs[i * 3]);
        m_ValueList[i].y = Util::Double(lBufPtrs[i * 3 + 1]);
        m_ValueList[i].z = Util::Double(lBufPtrs[i * 3 + 2]);
    }
}

// returns the degree of match to the stored values
// in this case this is the euclidean distance between the two vectors
double DataTargetVector::GetError(int valueListIndex)
{
    const double *r;
    Body *body;
    Geom *geom;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    UniversalJoint *universalJoint;
    PositionReporter *positionReporter;
    double err = 0;
    dVector3 v;
    if (valueListIndex < 0) valueListIndex = 0;
    if (valueListIndex >= m_ValueListLength)
    {
        std::cerr << "Warning: DataTargetVector::GetMatchValue valueListIndex out of range\n";
        return 0;
    }

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        r = body->GetPosition();
        err = (pgd::Vector(r[0], r[1], r[2]) - m_ValueList[valueListIndex]).Magnitude();
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        geom->GetWorldPosition(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
    }
    else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
    {
        hingeJoint->GetHingeAnchor(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
   }
    else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
    {
        ballJoint->GetBallAnchor(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
   }
    else if ((universalJoint = dynamic_cast<UniversalJoint *>(m_Target)) != 0)
    {
        universalJoint->GetUniversalAnchor(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
    }
    else if ((positionReporter = dynamic_cast<PositionReporter *>(m_Target)) != 0)
    {
        pgd::Vector vec = positionReporter->GetWorldPosition();
        err = (vec - m_ValueList[valueListIndex]).Magnitude();
    }
    else
    {
        std::cerr << "DataTargetVector target missing error " << m_Name << "\n";
    }
    return err;
}

// returns the degree of match to the stored values
// in this case this is the euclidean distance between the two vectors
double DataTargetVector::GetError(double time)
{
    const double *r;
    Body *body;
    Geom *geom;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    UniversalJoint *universalJoint;
    PositionReporter *positionReporter;
    double err = 0;
    dVector3 v;

    int index = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, time);
    if (index < 0) index = 0;
    if (index >= m_ValueListLength - 1)
    {
        std::cerr << "Warning: DataTargetVector::GetMatchValue index out of range\n";
        return 0;
    }
    int indexNext = index + 1;

    double interpX = Util::Interpolate(m_TargetTimeList[index], m_ValueList[index].x, m_TargetTimeList[indexNext], m_ValueList[indexNext].x, time);
    double interpY = Util::Interpolate(m_TargetTimeList[index], m_ValueList[index].y, m_TargetTimeList[indexNext], m_ValueList[indexNext].y, time);
    double interpZ = Util::Interpolate(m_TargetTimeList[index], m_ValueList[index].z, m_TargetTimeList[indexNext], m_ValueList[indexNext].z, time);
    pgd::Vector interpolatedTarget(interpX, interpY, interpZ);

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        r = body->GetPosition();
        err = (pgd::Vector(r[0], r[1], r[2]) - interpolatedTarget).Magnitude();
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        geom->GetWorldPosition(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - interpolatedTarget).Magnitude();
    }
    else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
    {
        hingeJoint->GetHingeAnchor(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - interpolatedTarget).Magnitude();
   }
    else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
    {
        ballJoint->GetBallAnchor(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - interpolatedTarget).Magnitude();
   }
    else if ((universalJoint = dynamic_cast<UniversalJoint *>(m_Target)) != 0)
    {
        universalJoint->GetUniversalAnchor(v);
        err = (pgd::Vector(v[0], v[1], v[2]) - interpolatedTarget).Magnitude();
    }
    else if ((positionReporter = dynamic_cast<PositionReporter *>(m_Target)) != 0)
    {
        pgd::Vector vec = positionReporter->GetWorldPosition();
        err = (vec - interpolatedTarget).Magnitude();
    }
    else
    {
        std::cerr << "DataTargetVector target missing error " << m_Name << "\n";
    }
    return err;
}

void DataTargetVector::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "NamedObject::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tTargetX\tTargetY\tTargetZ\tActualX\tActualY\tActualZ\tDistance\n";
        }
    }

    Body *body;
    Geom *geom;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    UniversalJoint *universalJoint;
    PositionReporter *positionReporter;
    const double *r = 0;
    double err = 0;
    dVector3 v;

    if (m_DumpStream)
    {
        int valueListIndex = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, m_simulation->GetTime());
        if ((body = dynamic_cast<Body *>(m_Target)) != 0)
        {
            r = body->GetPosition();
            err = (pgd::Vector(r[0], r[1], r[2]) - m_ValueList[valueListIndex]).Magnitude();
        }
        else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
        {
            geom->GetWorldPosition(v);
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
            r = v;
        }
        else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
        {
            hingeJoint->GetHingeAnchor(v);
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
            r = v;
        }
        else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
        {
            ballJoint->GetBallAnchor(v);
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
            r = v;
        }
        else if ((universalJoint = dynamic_cast<UniversalJoint *>(m_Target)) != 0)
        {
            universalJoint->GetUniversalAnchor(v);
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
            r = v;
        }
        else if ((positionReporter = dynamic_cast<PositionReporter *>(m_Target)) != 0)
        {
            pgd::Vector vec = positionReporter->GetWorldPosition();
            v[0] = vec.x; v[1] = vec.y; v[2] = vec.z;
            err = (pgd::Vector(v[0], v[1], v[2]) - m_ValueList[valueListIndex]).Magnitude();
            r = v;
        }

        *m_DumpStream << m_simulation->GetTime() <<
                "\t" << m_ValueList[valueListIndex].x << "\t" << m_ValueList[valueListIndex].y << "\t" << m_ValueList[valueListIndex].z <<
                "\t" << r[0] << "\t" << r[1] << "\t" << r[2] <<
                "\t" << err <<
                "\n";
    }
}

#ifdef USE_QT
void DataTargetVector::Draw(SimulationWindow *window)
{
    // perform late initialisation
    if (m_FirstDraw)
    {
        m_FirstDraw = false;
        if (mOnlyDrawNext)
        {
            m_physRep = new FacetedSphere(mRadius, 4);
            m_physRep->SetColour(m_Colour);
            m_physRep->setSimulationWindow(window);
        }
        else
        {
            m_physRep = new FacetedObject();
            m_physRep->SetColour(m_Colour);
            // m_physRep->SetBadMesh(true);
            m_physRep->setSimulationWindow(window);
            FacetedSphere *sphere = new FacetedSphere(mRadius, 4);
            pgd::Vector lastPosition, delta;
            for (int i = 0; i < m_ValueListLength; i++)
            {
                delta = m_ValueList[i] - lastPosition;
                lastPosition = m_ValueList[i];
                sphere->Move(delta.x, delta.y, delta.z);
                m_physRep->AddFacetedObject(sphere, false);
            }
            delete sphere;
        }

        if (m_axisSceneNode == 0)
        {
            // create meshes for the axes
            irr::scene::IMesh *mesh;
            irr::scene::IMeshSceneNode *sceneNode;
            const irr::scene::IGeometryCreator *geometryCreator = window->sceneManager()->getGeometryCreator();

            float axisSizeX = mRadius * 10;
            float axisSizeY = mRadius * 10;
            float axisSizeZ = mRadius * 10;
            float widthFraction = 0.1;
            mesh = geometryCreator->createCubeMesh(irr::core::vector3df(axisSizeX * widthFraction, axisSizeY * widthFraction, axisSizeZ * widthFraction));
            window->SetMeshColour(mesh, irr::video::SColor(255, 220, 220, 220));
            m_axisSceneNode = window->sceneManager()->addMeshSceneNode(mesh);
            m_axisSceneNode->setAutomaticCulling(irr::scene::EAC_OFF);

            irr::u32 tesselationCylinder = 64;
            irr::u32 tesselationCone = 128;
            // irr::f32 height = m_AxisSize[0];
            irr::f32 height = axisSizeX;
            irr::f32 cylinderHeight = height * (1 - 2 * widthFraction);
            irr::f32 widthCylinder = height * 0.25 * widthFraction;
            irr::f32 widthCone = height * widthFraction * 0.5;
            irr::video::SColor colorCylinder(255, 255, 0, 0); // red
            irr::video::SColor colorCone = colorCylinder;
            mesh = geometryCreator->createArrowMesh(tesselationCylinder, tesselationCone, height, cylinderHeight, widthCylinder, widthCone, colorCylinder, colorCone);
            window->SetMeshColour(mesh, irr::video::SColor(255, 255, 0, 0));
            sceneNode = window->sceneManager()->addMeshSceneNode(mesh, m_axisSceneNode);
            sceneNode->setAutomaticCulling(irr::scene::EAC_OFF);
            sceneNode->setRotation(irr::core::vector3df(0, 0, -90)); // rotates Y direction to X direction

            colorCylinder.set(255, 0, 255, 0); // green
            colorCone = colorCylinder;
            mesh = geometryCreator->createArrowMesh(tesselationCylinder, tesselationCone, height, cylinderHeight, widthCylinder, widthCone, colorCylinder, colorCone);
            window->SetMeshColour(mesh, irr::video::SColor(255, 0, 255, 0));
            sceneNode = window->sceneManager()->addMeshSceneNode(mesh, m_axisSceneNode);
            sceneNode->setAutomaticCulling(irr::scene::EAC_OFF);
            sceneNode->setRotation(irr::core::vector3df(0, 0, 0)); // rotates Y direction to Y direction

            colorCylinder.set(255, 0, 0, 255); // blue
            colorCone = colorCylinder;
            mesh = geometryCreator->createArrowMesh(tesselationCylinder, tesselationCone, height, cylinderHeight, widthCylinder, widthCone, colorCylinder, colorCone);
            window->SetMeshColour(mesh, irr::video::SColor(255, 0, 255, 0));
            sceneNode = window->sceneManager()->addMeshSceneNode(mesh, m_axisSceneNode);
            sceneNode->setAutomaticCulling(irr::scene::EAC_OFF);
            sceneNode->setRotation(irr::core::vector3df(90, 0, 0)); // rotates Y direction to Z direction

        }

    }

    // and draw the sphere(s)
    if (mOnlyDrawNext)
    {
        int valueListIndex = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, m_simulation->GetTime());
        if (valueListIndex < 0) valueListIndex = 0;
        else if (valueListIndex >= m_TargetTimeListLength) valueListIndex = m_TargetTimeListLength - 1;
        m_physRep->SetDisplayPosition(m_ValueList[valueListIndex].x, m_ValueList[valueListIndex].y, m_ValueList[valueListIndex].z);
        m_physRep->Draw();
    }
    else
    {
        m_physRep->Draw();

        int valueListIndex = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, m_simulation->GetTime());
        if (valueListIndex < 0) valueListIndex = 0;
        else if (valueListIndex >= m_TargetTimeListLength) valueListIndex = m_TargetTimeListLength - 1;
        m_axisSceneNode->setPosition(irr::core::vector3df(m_ValueList[valueListIndex].x, m_ValueList[valueListIndex].y, m_ValueList[valueListIndex].z));
    }
}
#endif
