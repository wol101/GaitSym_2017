/*
 *  Environment.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Mar 26 2005.
 *  Copyright (c) 2005 Bill Sellers. All rights reserved.
 *
 */


#include <ode/ode.h>

#ifdef USE_QT
#include "GLUtils.h"
#include "SimulationWindow.h"

// ruler definition
//static float gRulerMin = -500;
//static float gRulerMax = 500;
//static float gRulerTextSize = 0.1;
//static float gRulerTextInterval = 1.0;
//static float gRulerTickSize = 0.05;
//static float gRulerTickInterval = 0.5;

#endif

#include "Environment.h"
#include "Geom.h"
#include "PlaneGeom.h"


Environment::Environment()
{
#ifdef USE_QT
    m_axisSceneNode = 0;
#endif
}

Environment::~Environment()
{
    std::vector<Geom *>::const_iterator iter1;
    for (iter1=m_GeomList.begin(); iter1 != m_GeomList.end(); iter1++)
        delete *iter1;
}

void Environment::AddGeom(Geom *geom)
{
    m_GeomList.push_back(geom);
}

#ifdef USE_QT
void Environment::Draw(SimulationWindow *window)
{
    if (m_Visible == false) return;

    if (m_axisSceneNode == 0)
    {
        // create meshes for the axes
        irr::scene::IMesh *mesh;
        irr::scene::IMeshSceneNode *sceneNode;
        const irr::scene::IGeometryCreator *geometryCreator = window->sceneManager()->getGeometryCreator();

        float widthFraction = 0.1;
        mesh = geometryCreator->createCubeMesh(irr::core::vector3df(m_AxisSize[0] * widthFraction, m_AxisSize[0] * widthFraction, m_AxisSize[0] * widthFraction));
        window->SetMeshColour(mesh, irr::video::SColor(255, 220, 220, 220));
        m_axisSceneNode = window->sceneManager()->addMeshSceneNode(mesh);
        mesh->drop();
        m_axisSceneNode->setAutomaticCulling(irr::scene::EAC_OFF);

        irr::u32 tesselationCylinder = 64;
        irr::u32 tesselationCone = 128;
        irr::f32 height = m_AxisSize[0];
        irr::f32 cylinderHeight = height * (1 - 2 * widthFraction);
        irr::f32 widthCylinder = height * 0.25 * widthFraction;
        irr::f32 widthCone = height * widthFraction * 0.5;
        irr::video::SColor colorCylinder(255, 255, 0, 0); // red
        irr::video::SColor colorCone = colorCylinder;
        mesh = geometryCreator->createArrowMesh(tesselationCylinder, tesselationCone, height, cylinderHeight, widthCylinder, widthCone, colorCylinder, colorCone);
        window->SetMeshColour(mesh, irr::video::SColor(255, 255, 0, 0));
        sceneNode = window->sceneManager()->addMeshSceneNode(mesh, m_axisSceneNode);
        mesh->drop();
        sceneNode->setAutomaticCulling(irr::scene::EAC_OFF);
        sceneNode->setRotation(irr::core::vector3df(0, 0, -90)); // rotates Y direction to X direction

        colorCylinder.set(255, 0, 255, 0); // green
        colorCone = colorCylinder;
        mesh = geometryCreator->createArrowMesh(tesselationCylinder, tesselationCone, height, cylinderHeight, widthCylinder, widthCone, colorCylinder, colorCone);
        window->SetMeshColour(mesh, irr::video::SColor(255, 0, 255, 0));
        sceneNode = window->sceneManager()->addMeshSceneNode(mesh, m_axisSceneNode);
        mesh->drop();
        sceneNode->setAutomaticCulling(irr::scene::EAC_OFF);
        sceneNode->setRotation(irr::core::vector3df(0, 0, 0)); // rotates Y direction to Y direction

        colorCylinder.set(255, 0, 0, 255); // blue
        colorCone = colorCylinder;
        mesh = geometryCreator->createArrowMesh(tesselationCylinder, tesselationCone, height, cylinderHeight, widthCylinder, widthCone, colorCylinder, colorCone);
        window->SetMeshColour(mesh, irr::video::SColor(255, 0, 0, 255));
        sceneNode = window->sceneManager()->addMeshSceneNode(mesh, m_axisSceneNode);
        mesh->drop();
        sceneNode->setAutomaticCulling(irr::scene::EAC_OFF);
        sceneNode->setRotation(irr::core::vector3df(90, 0, 0)); // rotates Y direction to Z direction

    }

    // set the origin axis to 0,0,0
    m_axisSceneNode->setPosition(irr::core::vector3df(0, 0, 0));

//    // draw ruler
//    GLUtils::SetDrawColour(m_Colour.r, m_Colour.g, m_Colour.b, m_Colour.alpha);
//    char buffer[256];
//    float v;
//    float rotation[12];
//    float cosa = 0; // angle = +90 degrees
//    float sina = 1; // angle = +90 degrees
//    rotation[0] = 1; rotation[1] = 0;    rotation[2] = 0;
//    rotation[4] = 0; rotation[5] = cosa; rotation[6] = -sina;
//    rotation[8] = 0; rotation[9] = sina; rotation[10] = cosa;
//    for (v = gRulerMin; v <= gRulerMax; v += gRulerTextInterval)
//    {
//        sprintf(buffer, "%.1f", v);
//        GLUtils::OutputText(v, -2 * gRulerTextSize, 0, buffer, gRulerTextSize, rotation, 0);
//    }

//    for (v = gRulerMin; v <= gRulerMax; v += gRulerTickInterval)
//    {
//        GLUtils::DrawLine(v, 0, 0, v, 0, -gRulerTickSize);
//    }
//    // draw as 2 lines so we don't overwite the axis
//    GLUtils::DrawLine(gRulerMin, 0, 0, 0, 0, 0);
//    GLUtils::DrawLine(m_AxisSize[0], 0, 0, gRulerMax, 0, 0);

    // now draw the geoms
    for (unsigned int i = 0; i < m_GeomList.size(); i++)
        m_GeomList[i]->Draw(window);

}

#endif

void Environment::WriteToXMLStream(std::ostream &outputStream)
{
    outputStream << "<ENVIRONMENT";
    if (m_GeomList.size() > 0)
    {
        PlaneGeom *planeGeom = dynamic_cast<PlaneGeom *>(m_GeomList[0]);
        if (planeGeom)
        {
            dVector4 result;
            dGeomPlaneGetParams(planeGeom->GetGeomID(), result);
            outputStream << " Plane=\"" << result[0] << " " << result[1] << " " << result[2] << " " << result[3] << "\"";

#ifdef USE_QT
            if (planeGeom->m_texture)
            {
                outputStream << " TrackSensitivity=\"" << planeGeom->m_lowRange <<  " " << planeGeom->m_highRange << "\"";
                outputStream << " CheckerboardLow=\"" << planeGeom->m_checkerboardLow << "\"";
                outputStream << " CheckerboardHigh=\"" << planeGeom->m_checkerboardHigh << "\"";
                outputStream << " TrackPatch=\"" << planeGeom->m_trackPatchStartX << " " << planeGeom->m_trackPatchStartY << " " << planeGeom->m_trackPatchEndX << " "
                           << planeGeom->m_trackPatchEndY << " " << planeGeom->m_trackPatchResolutionX << " " << planeGeom->m_trackPatchResolutionY << " " "\"";
                outputStream << " TrackDrawThreshold=\"" << planeGeom->m_trackDrawThreshold << "\"";
            }
#endif
        }
    }
    outputStream << "/>\n";
}

