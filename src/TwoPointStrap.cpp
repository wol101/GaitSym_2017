/*
 *  TwoPointStrap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <iostream>
#include <vector>

#include <cmath>
#include <string.h>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include "TwoPointStrap.h"
#include "Body.h"
#include "PGDMath.h"
#include "DataFile.h"
#include "Simulation.h"
#include "DebugControl.h"

#ifdef USE_QT
#include "FacetedPolyline.h"
#endif

TwoPointStrap::TwoPointStrap()
{
    PointForce *origin = new PointForce();
    PointForce *insertion = new PointForce();
    m_PointForceList.push_back(origin);
    m_PointForceList.push_back(insertion);
}

TwoPointStrap::~TwoPointStrap()
{
}

void TwoPointStrap::SetOrigin(Body *body, dVector3 point)
{
    m_OriginBody = body;
    m_PointForceList[0]->body = m_OriginBody;
    memcpy(m_Origin, point, sizeof(m_Origin));
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
TwoPointStrap::SetOrigin(Body *body, const char *buf)
{
        int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;

    strcpy(lBuf, buf);
        int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
        if (count < 3)
    {
        std::cerr << "Error in TwoPointStrap::SetOrigin\n";
        return; // error condition
    }


        if (isalpha((int)*lBufPtrs[0]) == 0)
        {
                for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetOrigin(body, result);
                return;
        }

        if (count < 4)
    {
        std::cerr << "Error in TwoPointStrap::SetOrigin\n";
        return; // error condition
    }
        Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
        if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetOrigin(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in TwoPointStrap::SetOrigin\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetOrigin(body, pos);
}

void TwoPointStrap::SetInsertion(Body *body, dVector3 point)
{
    m_InsertionBody = body;
    m_PointForceList[1]->body = m_InsertionBody;
    memcpy(m_Insertion, point, sizeof(m_Insertion));
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
TwoPointStrap::SetInsertion(Body *body, const char *buf)
{
        int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;

    strcpy(lBuf, buf);
        int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
        if (count < 3)
    {
        std::cerr << "Error in TwoPointStrap::SetInsertion\n";
        return; // error condition
    }

        if (isalpha((int)*lBufPtrs[0]) == 0)
        {
                for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetInsertion(body, result);
                return;
        }

        if (count < 4)
    {
        std::cerr << "Error in TwoPointStrap::SetInsertion\n";
        return; // error condition
    }
        Body *theBody = m_simulation->GetBody(lBufPtrs[0]);
        if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetInsertion(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in TwoPointStrap::SetInsertion\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetInsertion(body, pos);
}

void TwoPointStrap::Calculate(double deltaT)
{
    PointForce *theOrigin = m_PointForceList[0];
    PointForce *theInsertion = m_PointForceList[1];

    // calculate the world positions
    dBodyGetRelPointPos(m_OriginBody->GetBodyID(), m_Origin[0], m_Origin[1], m_Origin[2],
                        theOrigin->point);
    dBodyGetRelPointPos(m_InsertionBody->GetBodyID(), m_Insertion[0], m_Insertion[1], m_Insertion[2],
                        theInsertion->point);

    // calculate the vector from the origin to the insertion
    dVector3 line;
    line[0] = theInsertion->point[0] - theOrigin->point[0];
    line[1] = theInsertion->point[1] - theOrigin->point[1];
    line[2] = theInsertion->point[2] - theOrigin->point[2];

    // calculate the length and velocity
    m_LastLength = m_Length;
    m_Length = sqrt(line[0]*line[0] + line[1]*line[1] + line[2]*line[2]);
    if (deltaT != 0.0) m_Velocity = (m_Length - m_LastLength) / deltaT;
    else m_Velocity = 0;

    // normalise the direction vector
    line[0] /= m_Length;
    line[1] /= m_Length;
    line[2] /= m_Length;

    memcpy(theOrigin->vector, line, sizeof(theOrigin->vector));

    // simply reverse the direction for the insertion
    line[0] = -line[0];
    line[1] = -line[1];
    line[2] = -line[2];

    memcpy(theInsertion->vector, line, sizeof(theInsertion->vector));

    if (gDebug == StrapDebug)
    {
        pgd::Vector totalF(0, 0, 0);
        for (unsigned int i = 0; i < m_PointForceList.size(); i++)
        {
            *gDebugStream << "TwoPointStrap::Calculate " <<
            *m_PointForceList[i]->body->GetName() << " " <<
            m_PointForceList[i]->point[0] << " " << m_PointForceList[i]->point[1] << " " << m_PointForceList[i]->point[2] << " " <<
            m_PointForceList[i]->vector[0] << " " << m_PointForceList[i]->vector[1] << " " << m_PointForceList[i]->vector[2] << "\n";
            totalF.x += m_PointForceList[i]->vector[0]; totalF.y += m_PointForceList[i]->vector[1]; totalF.z += m_PointForceList[i]->vector[2];
        }
        *gDebugStream << "Total F " << totalF.x << " " << totalF.y << " " << totalF.z << "\n";
    }
}

int TwoPointStrap::SanityCheck(Strap *otherStrap, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight)
{
    const double epsilon = 1e-10;

    TwoPointStrap *other = dynamic_cast<TwoPointStrap *>(otherStrap);
    if (other == 0) return __LINE__;

    // first check attachment errors
    switch (axis)
    {
    case XAxis:
        if (fabs(this->m_Origin[0] + other->m_Origin[0]) > epsilon) return __LINE__;
        if (fabs(this->m_Origin[1] - other->m_Origin[1]) > epsilon) return __LINE__;
        if (fabs(this->m_Origin[2] - other->m_Origin[2]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[0] + other->m_Insertion[0]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[1] - other->m_Insertion[1]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[2] - other->m_Insertion[2]) > epsilon) return __LINE__;
        break;

    case YAxis:
        if (fabs(this->m_Origin[0] - other->m_Origin[0]) > epsilon) return __LINE__;
        if (fabs(this->m_Origin[1] + other->m_Origin[1]) > epsilon) return __LINE__;
        if (fabs(this->m_Origin[2] - other->m_Origin[2]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[0] - other->m_Insertion[0]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[1] + other->m_Insertion[1]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[2] - other->m_Insertion[2]) > epsilon) return __LINE__;
        break;

    case ZAxis:
        if (fabs(this->m_Origin[0] - other->m_Origin[0]) > epsilon) return __LINE__;
        if (fabs(this->m_Origin[1] - other->m_Origin[1]) > epsilon) return __LINE__;
        if (fabs(this->m_Origin[2] + other->m_Origin[2]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[0] - other->m_Insertion[0]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[1] - other->m_Insertion[1]) > epsilon) return __LINE__;
        if (fabs(this->m_Insertion[2] + other->m_Insertion[2]) > epsilon) return __LINE__;
        break;
    }

    // now check for left to right crossover errors
    if (this->m_Name.find(sanityCheckLeft) != std::string::npos)
    {
        if (m_OriginBody->GetName()->find(sanityCheckRight) != std::string::npos) return __LINE__;
        if (m_InsertionBody->GetName()->find(sanityCheckRight) != std::string::npos) return __LINE__;
    }
    if (this->m_Name.find(sanityCheckRight) != std::string::npos)
    {
        if (m_OriginBody->GetName()->find(sanityCheckLeft) != std::string::npos) return __LINE__;
        if (m_InsertionBody->GetName()->find(sanityCheckLeft) != std::string::npos) return __LINE__;
    }

    return 0;
}

#ifdef USE_QT
void
TwoPointStrap::Draw(SimulationWindow *window)
{
    const int kSides = 128;
    FacetedPolyline *facetedPolyline;
    unsigned int i;

    if (m_simulation->GetTime() != m_LastDrawTime)
    {
        m_LastDrawTime = m_simulation->GetTime();
        std::vector<FacetedObject *>::const_iterator iterFO;
        for (iterFO = m_DrawList.begin(); iterFO != m_DrawList.end(); iterFO++)
            delete *iterFO;
        m_DrawList.clear();

        std::vector<pgd::Vector> polyline;
        polyline.push_back(pgd::Vector(m_PointForceList[0]->point[0], m_PointForceList[0]->point[1], m_PointForceList[0]->point[2]));
        polyline.push_back(pgd::Vector(m_PointForceList[1]->point[0], m_PointForceList[1]->point[1], m_PointForceList[1]->point[2]));
        facetedPolyline = new FacetedPolyline(&polyline, m_Radius, kSides);
        facetedPolyline->SetColour(m_Colour);
        facetedPolyline->setSimulationWindow(window);
        facetedPolyline->SetVisible(m_Visible);
        m_DrawList.push_back(facetedPolyline);

        if (m_drawMuscleForces)
        {
            for (i = 0; i < m_PointForceList.size(); i++)
            {
                polyline.clear();
                pgd::Vector f = pgd::Vector(m_PointForceList[i]->vector[0], m_PointForceList[i]->vector[1], m_PointForceList[i]->vector[2]) * m_Tension * m_ForceScale;
                polyline.push_back(pgd::Vector(m_PointForceList[i]->point[0], m_PointForceList[i]->point[1], m_PointForceList[i]->point[2]));
                polyline.push_back(pgd::Vector(m_PointForceList[i]->point[0], m_PointForceList[i]->point[1], m_PointForceList[i]->point[2]) + f);
                facetedPolyline = new FacetedPolyline(&polyline, m_ForceRadius, kSides);
                facetedPolyline->SetColour(m_Colour);
                facetedPolyline->setSimulationWindow(window);
                facetedPolyline->SetVisible(m_drawMuscleForces);
                m_DrawList.push_back(facetedPolyline);
            }
        }
    }

    for (i = 0; i < m_DrawList.size(); i++)
        m_DrawList[i]->Draw();
}
#endif



